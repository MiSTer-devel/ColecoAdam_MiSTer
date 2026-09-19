/** colem_ref: headless ColEm 5.6 platform layer ************/
/** Replaces ColEm/Unix/Unix.c + EMULib/Unix/LibUnix.c.     **/
/** No X11, no audio. Dumps cropped 256x192 frames as PPM   **/
/** and PNG, and scripts controller 1 input by frame.       **/
/*************************************************************/
#include "Coleco.h"
#include "EMULib.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <zlib.h>

#define WIDTH   272
#define HEIGHT  200
/* Active-area origin, from DRV9918.c RefreshLine1/2/3:     */
/*   XBuf + Width*(Y+(Height-192)/2) + Width/2-128          */
#define CROP_X  (WIDTH/2-128)
#define CROP_Y  ((HEIGHT-192)/2)

extern const char *BiosName;          /* Added to Coleco.c copy */

static pixel ScrData[WIDTH*HEIGHT];
static int FrameCount = 0;            /* Completed frames       */
static int MaxFrames  = 0;
static int Every      = 0;
static int Shots[1024];
static int ShotCount  = 0;
static const char *OutDir = 0;

typedef struct { unsigned int Bits; int Keypad; int Start; int Dur; } Press;
static Press Presses[256];
static int PressCount = 0;

/* Adam: media inserted on the first frame, keys typed by frame */
#define KEY_GAP 8                             /* Frames per typed key, as in sim_main.cpp */
extern const char *WriterName,*EosName;      /* Added to Coleco.c copy */
static const char *DiskFile[MAX_DISKS],*TapeFile[MAX_TAPES];
typedef struct { unsigned int Key; int Frame; } Stroke;
static Stroke Strokes[4096];
static int StrokeCount = 0;

/** Platform hooks required by Coleco.c *********************/

int SetColor(byte N,byte R,byte G,byte B)
{
  return(((int)R<<16)|((int)G<<8)|B);
}

unsigned int Mouse(void) { return(0); }

/* Called at the end of frame FrameCount (after RefreshScreen). */
/* The returned state is what the game sees during the next     */
/* frame, so a press starting at frame F is returned from the   */
/* end of frame F-1 and is visible during frames F..F+Dur-1.    */
unsigned int Joystick(void)
{
  unsigned int J = 0;
  int K = -1,I,F = FrameCount+1;

  for(I=0;I<PressCount;++I)
    if((F>=Presses[I].Start)&&(F<Presses[I].Start+Presses[I].Dur))
    {
      J |= Presses[I].Bits;
      if(Presses[I].Keypad>=0) K=Presses[I].Keypad;
    }

  /* Keypad is a 4-bit code: only one key can be down at a time */
  if(K>=0) J=(J&~JST_KEYPAD)|K;
  return(J);
}

static void Put32(unsigned char *P,unsigned int V)
{
  P[0]=V>>24;P[1]=V>>16;P[2]=V>>8;P[3]=V;
}

static void PNGChunk(FILE *F,const char *Type,const unsigned char *Data,unsigned int Len)
{
  unsigned char B[4];
  uLong CRC;

  Put32(B,Len);
  fwrite(B,1,4,F);
  fwrite(Type,1,4,F);
  if(Len) fwrite(Data,1,Len,F);
  CRC = crc32(0L,(const Bytef *)Type,4);
  if(Len) CRC=crc32(CRC,Data,Len);
  Put32(B,(unsigned int)CRC);
  fwrite(B,1,4,F);
}

static void SaveFrame(int N)
{
  static unsigned char RGBBuf[192*256*3];
  static unsigned char Raw[192*(1+256*3)];
  unsigned char IHDR[13],*Z,*P;
  char Name[4096];
  uLongf ZLen;
  FILE *F;
  int X,Y;

  /* Crop active area */
  for(Y=0,P=RGBBuf;Y<192;++Y)
    for(X=0;X<256;++X)
    {
      pixel C = ScrData[(Y+CROP_Y)*WIDTH+X+CROP_X];
      *P++=(C>>16)&0xFF;*P++=(C>>8)&0xFF;*P++=C&0xFF;
    }

  /* PPM */
  snprintf(Name,sizeof(Name),"%s/frame_%05d.ppm",OutDir,N);
  if(!(F=fopen(Name,"wb"))) { fprintf(stderr,"Cannot write %s\n",Name);exit(3); }
  fprintf(F,"P6\n256 192\n255\n");
  fwrite(RGBBuf,1,sizeof(RGBBuf),F);
  fclose(F);

  /* PNG: RGB8, filter 0 on every row */
  for(Y=0,P=Raw;Y<192;++Y)
  {
    *P++=0;
    memcpy(P,RGBBuf+Y*256*3,256*3);
    P+=256*3;
  }
  ZLen = compressBound(sizeof(Raw));
  if(!(Z=malloc(ZLen))) exit(3);
  if(compress2(Z,&ZLen,Raw,sizeof(Raw),6)!=Z_OK) { fprintf(stderr,"zlib error\n");exit(3); }

  Put32(IHDR,256);Put32(IHDR+4,192);
  IHDR[8]=8;IHDR[9]=2;IHDR[10]=0;IHDR[11]=0;IHDR[12]=0;

  snprintf(Name,sizeof(Name),"%s/frame_%05d.png",OutDir,N);
  if(!(F=fopen(Name,"wb"))) { fprintf(stderr,"Cannot write %s\n",Name);exit(3); }
  fwrite("\x89PNG\r\n\x1a\n",1,8,F);
  PNGChunk(F,"IHDR",IHDR,13);
  PNGChunk(F,"IDAT",Z,(unsigned int)ZLen);
  PNGChunk(F,"IEND",0,0);
  fclose(F);
  free(Z);
}

void RefreshScreen(void *Buffer,int Width,int Height)
{
  int I,Want;

  ++FrameCount;

  /* Insert Adam media once the ROMs are loaded; drive A: resets into Adam mode */
  if(FrameCount==1)
  {
    for(I=0;I<MAX_DISKS;++I) if(DiskFile[I]) ChangeDisk(I,DiskFile[I]);
    for(I=0;I<MAX_TAPES;++I) if(TapeFile[I]) ChangeTape(I,TapeFile[I]);
  }

  /* A key typed at frame F is visible to the game from frame F */
  for(I=0;I<StrokeCount;++I) if(Strokes[I].Frame==FrameCount+1) PutKBD(Strokes[I].Key);

  Want = Every>0 && !(FrameCount%Every);
  for(I=0;!Want&&(I<ShotCount);++I) Want=Shots[I]==FrameCount;
  if(Want) SaveFrame(FrameCount);

  if(FrameCount>=MaxFrames) ExitNow=1;
}

/** Stubs for EMULib platform functions ********************/
/* Only what the linked emulation core references.          */

unsigned int GetFreeAudio(void)                  { return(0); }
unsigned int GetTotalAudio(void)                 { return(0); }
unsigned int WriteAudio(sample *Data,unsigned int Length) { return(Length); }
int PauseAudio(int Switch)                       { return(0); }
unsigned int InitAudio(unsigned int Rate,unsigned int Latency) { return(0); }
void TrashAudio(void)                            { }

/** Command line ********************************************/

static int ParseKey(const char *S,unsigned int *Bits,int *Keypad)
{
  static const struct { const char *Name;unsigned int Bits;int Keypad; } Keys[] =
  {
    { "0",0,JST_0 },{ "1",0,JST_1 },{ "2",0,JST_2 },{ "3",0,JST_3 },
    { "4",0,JST_4 },{ "5",0,JST_5 },{ "6",0,JST_6 },{ "7",0,JST_7 },
    { "8",0,JST_8 },{ "9",0,JST_9 },{ "star",0,JST_STAR },{ "pound",0,JST_POUND },
    { "up",JST_UP,-1 },{ "down",JST_DOWN,-1 },{ "left",JST_LEFT,-1 },{ "right",JST_RIGHT,-1 },
    { "fire1",JST_FIREL,-1 },{ "fire2",JST_FIRER,-1 },
    { 0,0,0 }
  };
  int J;

  for(J=0;Keys[J].Name;++J)
    if(!strcmp(S,Keys[J].Name)) { *Bits=Keys[J].Bits;*Keypad=Keys[J].Keypad;return(1); }
  return(0);
}

/* Adam key names, shared with the core's --key (sim_main.cpp) */
static int AdamKeyName(const char *S)
{
  static const struct { const char *Name;int Code; } Keys[] =
  {
    { "enter",KEY_ENTER },{ "esc",KEY_ESC },{ "bs",KEY_BS },{ "tab",KEY_TAB },{ "space",' ' },
    { "up",KEY_UP },{ "down",KEY_DOWN },{ "left",KEY_LEFT },{ "right",KEY_RIGHT },{ "home",KEY_HOME },
    { "f1",KEY_F1 },{ "f2",KEY_F2 },{ "f3",KEY_F3 },{ "f4",KEY_F4 },{ "f5",KEY_F5 },{ "f6",KEY_F6 },
    { "undo",KEY_UNDO },{ "wildcard",KEY_WILDCARD },{ "move",KEY_MOVE },{ "store",KEY_STORE },
    { "insert",KEY_INS },{ "print",KEY_PRINT },{ "clear",KEY_CLEAR },{ "delete",KEY_DEL },
    { 0,0 }
  };
  int J;

  for(J=0;Keys[J].Name;++J)
    if(!strcmp(S,Keys[J].Name)) return(Keys[J].Code);
  return(-1);
}

/* PutKBD() lowercases letters unless SHIFT is given */
static unsigned int AdamKeyChar(int C)
{
  if(C=='\n') return(KEY_ENTER);
  if((C>='A')&&(C<='Z')) return(KEY_SHIFT|(C-'A'+'a'));
  return((unsigned char)C);
}

static void Usage(const char *Prog)
{
  fprintf(stderr,
    "Usage: %s [--cart FILE] --bios FILE --frames N [--shots F1,F2,...] [--every K]\n"
    "          --outdir DIR [--press KEY@FRAME[:DUR]]...\n"
    "          [--adam --writer FILE --eos FILE] [--disk N FILE] [--tape N FILE]\n"
    "          [--type TEXT@FRAME] [--key NAME@FRAME]\n"
    "KEY: 0-9 star pound up down left right fire1 fire2\n"
    "NAME: enter esc bs tab space up down left right home f1-f6 undo wildcard\n"
    "      move store insert print clear delete\n",Prog);
  exit(2);
}

int main(int argc,char *argv[])
{
  const char *Cart = 0,*Bios = 0;
  int Adam = 0;
  unsigned char Hdr[2];
  FILE *F;
  int N;

  for(N=1;N<argc;++N)
  {
    const char *A = argv[N];
    const char *V = N+1<argc? argv[N+1]:0;

    if(!strcmp(A,"--cart")&&V)        { Cart=V;++N; }
    else if(!strcmp(A,"--bios")&&V)   { Bios=V;++N; }
    else if(!strcmp(A,"--frames")&&V) { MaxFrames=atoi(V);++N; }
    else if(!strcmp(A,"--every")&&V)  { Every=atoi(V);++N; }
    else if(!strcmp(A,"--outdir")&&V) { OutDir=V;++N; }
    else if(!strcmp(A,"--shots")&&V)
    {
      char *Copy = strdup(V),*T;
      for(T=strtok(Copy,",");T&&(ShotCount<1024);T=strtok(0,",")) Shots[ShotCount++]=atoi(T);
      free(Copy);
      ++N;
    }
    else if(!strcmp(A,"--press")&&V)
    {
      char Key[32];
      int Start,Dur = 8;
      const char *At = strchr(V,'@');
      if(!At||(At-V)>=(int)sizeof(Key)||(PressCount>=256)) Usage(argv[0]);
      memcpy(Key,V,At-V);Key[At-V]='\0';
      if(sscanf(At+1,"%d:%d",&Start,&Dur)<1) Usage(argv[0]);
      if(!ParseKey(Key,&Presses[PressCount].Bits,&Presses[PressCount].Keypad))
      { fprintf(stderr,"Unknown key '%s'\n",Key);Usage(argv[0]); }
      Presses[PressCount].Start = Start;
      Presses[PressCount].Dur   = Dur;
      ++PressCount;
      ++N;
    }
    else if(!strcmp(A,"--adam"))       { Adam=1; }
    else if(!strcmp(A,"--writer")&&V) { WriterName=V;++N; }
    else if(!strcmp(A,"--eos")&&V)    { EosName=V;++N; }
    else if((!strcmp(A,"--disk")||!strcmp(A,"--tape"))&&V&&(N+2<argc))
    {
      int D = atoi(V)-1;
      if((D<0)||(D>=(A[2]=='d'? MAX_DISKS:MAX_TAPES))) Usage(argv[0]);
      if(A[2]=='d') DiskFile[D]=argv[N+2]; else TapeFile[D]=argv[N+2];
      N+=2;
    }
    else if((!strcmp(A,"--type")||!strcmp(A,"--key"))&&V)
    {
      const char *At = strrchr(V,'@');
      int Frame,Code,J,Slot;

      if(!At||(sscanf(At+1,"%d",&Frame)!=1)) Usage(argv[0]);
      if(A[2]=='k')
      {
        char Name[32];
        if((At-V)>=(int)sizeof(Name)) Usage(argv[0]);
        memcpy(Name,V,At-V);Name[At-V]='\0';
        if((Code=AdamKeyName(Name))<0) { fprintf(stderr,"Unknown key '%s'\n",Name);Usage(argv[0]); }
        if(StrokeCount<4096) { Strokes[StrokeCount].Key=Code;Strokes[StrokeCount++].Frame=Frame; }
      }
      else
        for(J=0,Slot=0;(V+J<At)&&(StrokeCount<4096);++J,++Slot)
        {
          int C = V[J];
          /* "\n" types Return */
          if((C=='\\')&&(V+J+1<At)&&(V[J+1]=='n')) { C='\n';++J; }
          Strokes[StrokeCount].Key   = AdamKeyChar(C);
          Strokes[StrokeCount].Frame = Frame+Slot*KEY_GAP;
          ++StrokeCount;
        }
      ++N;
    }
    else Usage(argv[0]);
  }

  if((!Cart&&!Adam)||!Bios||!OutDir||(MaxFrames<=0)) Usage(argv[0]);

  /* StartColeco() silently runs the bare BIOS if the cart fails */
  /* to load, so reject files ColEm would not accept up front.   */
  if(Cart)
  {
    if(!(F=fopen(Cart,"rb"))||(fread(Hdr,1,2,F)!=2))
    { fprintf(stderr,"Cannot read cartridge %s\n",Cart);return(2); }
    fclose(F);
  }

  /* ...and it silently falls back to ColecoVision without both Adam ROMs */
  if(Adam)
  {
    if(!WriterName||!EosName) { fprintf(stderr,"--adam needs --writer and --eos\n");return(2); }
    if(!(F=fopen(WriterName,"rb"))) { fprintf(stderr,"Cannot read %s\n",WriterName);return(2); }
    fclose(F);
    if(!(F=fopen(EosName,"rb"))) { fprintf(stderr,"Cannot read %s\n",EosName);return(2); }
    fclose(F);
  }
  for(N=0;N<MAX_DISKS;++N)
    if(DiskFile[N]&&!(F=fopen(DiskFile[N],"rb"))) { fprintf(stderr,"Cannot read %s\n",DiskFile[N]);return(2); }
    else if(DiskFile[N]) fclose(F);
  for(N=0;N<MAX_TAPES;++N)
    if(TapeFile[N]&&!(F=fopen(TapeFile[N],"rb"))) { fprintf(stderr,"Cannot read %s\n",TapeFile[N]);return(2); }
    else if(TapeFile[N]) fclose(F);
  if(!(F=fopen(Bios,"rb"))) { fprintf(stderr,"Cannot read BIOS %s\n",Bios);return(2); }
  fclose(F);
  if(mkdir(OutDir,0755)&&(errno!=EEXIST))
  { fprintf(stderr,"Cannot create %s\n",OutDir);return(2); }

  /* Plain NTSC ColecoVision, default palette, draw every frame */
  Verbose   = 0;
  UPeriod   = 100;
  Mode      = CV_NTSC|CV_PALETTE0|(Adam? CV_ADAM:0);
  SndName   = 0;
  PrnName   = "/dev/null";  /* SmartWriter prints as you type; keep stdout for the summary */
  PalName   = 0;
  HomeDir   = 0;
  BiosName  = Bios;
  ScrBuffer = ScrData;
  ScrWidth  = WIDTH;
  ScrHeight = HEIGHT;

  if(!StartColeco(Cart)) { fprintf(stderr,"StartColeco() failed\n");return(1); }

  printf("frames=%d crc=%X mode=%X\n",FrameCount,CartCRC(),Mode);
  return(FrameCount>=MaxFrames? 0:1);
}
