#include <verilated.h>
//#include "verilated_fst_sc.h"
#include "Vemu.h"

#include "imgui.h"
#include "implot.h"
#ifndef _MSC_VER
#include <stdio.h>
#include <SDL.h>
#include <SDL_opengl.h>
#else
#define WIN32
#include <dinput.h>
#endif

#include "sim_console.h"
#include "sim_bus.h"
#include "sim_blkdevice.h"
#include "sim_video.h"
#include "sim_audio.h"
#include "sim_input.h"
#include "sim_clock.h"

#include "../imgui/imgui_memory_editor.h"
#include "../imgui/ImGuiFileDialog.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <iterator>
#include <string>
#include <iomanip>
using namespace std;

// Simulation control
// ------------------
int initialReset = 48;
bool run_enable = 1;
bool adam_mode= 1;
int batchSize = 150000;
//int batchSize = 100;
bool single_step = 0;
bool multi_step = 0;
int multi_step_amount = 1024;

// Debug GUI
// ---------
const char* windowTitle = "Verilator Sim: Adam";
const char* windowTitle_Control = "Simulation control";
const char* windowTitle_DebugLog = "Debug log";
const char* windowTitle_Video = "VGA output";
const char* windowTitle_Audio = "Audio output";
bool showDebugLog = true;
DebugConsole console;
MemoryEditor mem_edit;

std::string tracefilename = "traces/adam.tr";

// HPS emulator
// ------------
SimBus bus(console);
SimBlockDevice blockdevice(console);

// Input handling
// --------------
SimInput input(13, console);
const int input_right = 0;
const int input_left = 1;
const int input_down = 2;
const int input_up = 3;
const int input_a = 4;
const int input_b = 5;
const int input_x = 6;
const int input_y = 7;
const int input_l = 8;
const int input_r = 9;
const int input_select = 10;
const int input_start = 11;
const int input_menu = 12;

// Video
// -----
#define VGA_WIDTH 320
#define VGA_HEIGHT 240
#define VGA_ROTATE 0  // 90 degrees anti-clockwise
#define VGA_SCALE_X vga_scale
#define VGA_SCALE_Y vga_scale
SimVideo video(VGA_WIDTH, VGA_HEIGHT, VGA_ROTATE);
float vga_scale = 2.5;

// Verilog module
// --------------
Vemu* top = NULL;

vluint64_t main_time = 0;	// Current simulation time.
double sc_time_stamp() {	// Called by $time in Verilog.
        return main_time;
}

int clk_sys_freq = 24000000;
SimClock clk_sys(1);

int soft_reset=0;
vluint64_t soft_reset_time=0;


// ADAM GLOBALS
#include "Coleco.h"
byte *ROMPage[8];              /* 8x8kB read-only (ROM) pages   */
byte *RAMPage[8];              /* 8x8kB read-write (RAM) pages  */
byte Port60=1;                   /* Adam port 0x60-0x7F (memory)  */

// MAME debug log
//#define CPU_DEBUG

#ifdef CPU_DEBUG
bool log_instructions = true;
bool stop_on_log_mismatch = true;

std::vector<std::string> log_mame;
std::vector<std::string> log_cpu;
long log_index;
unsigned int ins_count = 0;






// CPU debug
bool cpu_sync;
bool cpu_sync_last;
std::vector<std::vector<std::string> > opcodes;
std::map<std::string, std::string> opcode_lookup;

bool writeLog(const char* line)
{
        // Write to cpu log
        log_cpu.push_back(line);

        // Compare with MAME log
        bool match = true;
        ins_count++;

        std::string c_line = std::string(line);
        std::string c = "%d > " + c_line + " ";

        char buf[6];
#if 0
        unsigned char in1 = top->emu__DOT__system__DOT__in_p1_data;
        sprintf(buf, " %02X", in1);
        c.append(buf);
        unsigned char in2 = top->emu__DOT__system__DOT__in_p2_data;
        sprintf(buf, " %02X", in2);
        c.append(buf);
        unsigned char in3 = top->emu__DOT__system__DOT__in_p3_data;
        sprintf(buf, " %02X", in3);
        c.append(buf);
        unsigned char in4 = top->emu__DOT__system__DOT__in_p4_data;
        sprintf(buf, " %02X", in4);
        c.append(buf);
#endif
        if (log_index < log_mame.size()) {
                std::string m_line = log_mame.at(log_index);

                std::string m_line_lower = m_line.c_str();
                for (auto& c : m_line_lower) { c = tolower(c); }
                std::string c_line_lower = c_line.c_str();
                for (auto& c : c_line_lower) { c = tolower(c); }

                if (stop_on_log_mismatch && m_line_lower != c_line_lower) {
                        console.AddLog("DIFF at %d", log_index);
                        match = false;
                        run_enable = 0;
                }
                if (log_instructions) {
                        console.AddLog(c.c_str(), ins_count);
                        std::string m = "MAME > " + m_line;
                        console.AddLog(m.c_str());
                }
        }
        else {
                console.AddLog("MAME OUT");
                run_enable = 0;
        }

        log_index++;
        return match;

}

void loadOpcodes()
{
        std::string fileName = "z80_opcodes.csv";

        std::string                           header;
        std::ifstream                         reader(fileName);
        if (reader.is_open()) {
                std::string line, column, id;
                std::getline(reader, line);
                header = line;
                while (std::getline(reader, line)) {
                        std::stringstream        ss(line);
                        std::vector<std::string> columns;
                        bool                     withQ = false;
                        std::string              part{ "" };
                        while (std::getline(ss, column, ',')) {
                                auto pos = column.find("\"");
                                if (pos < column.length()) {
                                        withQ = !withQ;
                                        part += column.substr(0, pos);
                                        column = column.substr(pos + 1, column.length());
                                }
                                if (!withQ) {
                                        column += part;
                                        columns.emplace_back(std::move(column));
                                        part = "";
                                }
                                else {
                                        part += column + ",";
                                }
                        }
                        opcodes.push_back(columns);
                        opcode_lookup[columns[0]] = columns[1];
                }
        }
};

std::string int_to_hex(unsigned char val)
{
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(2) << std::hex << (val | 0);
        return ss.str();
}

std::string get_opcode(int ir, int ir_ext)
{
        std::string hex = "0x";
        if (ir_ext > 0) {
                hex.append(int_to_hex(ir_ext));
        }
        hex.append(int_to_hex(ir));
        if (opcode_lookup.find(hex) != opcode_lookup.end()) {
                return opcode_lookup[hex];
        }
        else
        {
                hex.append(" - MISSING OPCODE");
                return hex;
        }
}

bool hasEnding(std::string const& fullString, std::string const& ending) {
        if (fullString.length() >= ending.length()) {
                return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
        }
        else {
                return false;
        }
}
std::string last_log;
//
unsigned short last_pc;
unsigned short last_last_pc;
unsigned char last_mreq;

unsigned short active_pc;
unsigned char active_ir;
unsigned char active_ir_ext;
bool active_ir_valid = false;

const int ins_size = 48;
int ins_index = 0;
int ins_pc[ins_size];
int ins_in[ins_size];
int ins_ma[ins_size];
unsigned char active_ins = 0;

bool vbl_last;
bool rom_read_last;
#endif

// Audio
// -----
//#define DISABLE_AUDIO
#ifndef DISABLE_AUDIO
SimAudio audio(clk_sys_freq, false);
#endif

// Reset simulation variables and clocks
void resetSim() {
        main_time = 0;
        top->reset = 1;
        clk_sys.Reset();
}

int verilate() {

        if (!Verilated::gotFinish()) {
                if (soft_reset){
                        fprintf(stderr,"soft_reset.. in gotFinish\n");
                        top->soft_reset = 1;
                        soft_reset=0;
                        soft_reset_time=0;
                        fprintf(stderr,"turning on %x\n",top->soft_reset);
                }
                if (clk_sys.IsRising()) {
                        soft_reset_time++;
                }
                if (soft_reset_time==initialReset) {
                        top->soft_reset = 0;
                        fprintf(stderr,"turning off %x\n",top->soft_reset);
                        fprintf(stderr,"soft_reset_time %ld initialReset %x\n",soft_reset_time,initialReset);
                }

                // Assert reset during startup
                if (main_time < initialReset) { top->reset = 1; }
                // Deassert reset after startup
                if (main_time == initialReset) { top->reset = 0; }

                // Clock dividers
                clk_sys.Tick();

                // Set system clock in core
                top->clk_sys = clk_sys.clk;
                top->adam = adam_mode;

                // Simulate both edges of system clock
                if (clk_sys.clk != clk_sys.old) {
                  if (clk_sys.IsRising() && *bus.ioctl_download!=1	) {
                    //printf("BeforeEval\n");
                    blockdevice.BeforeEval(main_time);
                  }
                        if (clk_sys.clk) {
                                input.BeforeEval();
                                bus.BeforeEval();
                        }
                        top->eval();
                        if (clk_sys.clk) { bus.AfterEval(); blockdevice.AfterEval(); }
                }

#ifndef DISABLE_AUDIO
                if (clk_sys.IsRising())
                {
                        audio.Clock(top->AUDIO_L, top->AUDIO_R);
                }
#endif

                // Output pixels on rising edge of pixel clock
                if (clk_sys.IsRising() && top->CE_PIXEL ) {
                        uint32_t colour = 0xFF000000 | top->VGA_B << 16 | top->VGA_G << 8 | top->VGA_R;
                        video.Clock(top->VGA_HB, top->VGA_VB, top->VGA_HS, top->VGA_VS, colour);
                }

                if (clk_sys.IsRising()) {


                  /*A
                        /* ADAM NET */
                  //                        if (top->emu__DOT__console__DOT__adamnet__DOT__adam_reset_pcb_n_i == 0) // negative signal
                  //     {
                  //             printf("ResetPCB from sim_main\n");
                  //            ResetPCB();
                  //    }
                  //    /* if we are writing -- check it ? */
                  //    if (top->emu__DOT__console__DOT__adamnet__DOT__z80_wr)
                  //    {
                  //
                  //            word A = top->emu__DOT__console__DOT__adamnet__DOT__z80_addr;
                  //            word V = top->emu__DOT__console__DOT__adamnet__DOT__z80_data_wr;
                  //            if(PCBTable[A]) {
                  //                    printf("z80_wr, WritePCB A %x V %x %x\n",A,V,PCBTable[A]);
                  //                    WritePCB(A,V);
                  //            }
                  //    }
                  //    if (top->emu__DOT__console__DOT__adamnet__DOT__z80_rd)
                  //    {
                  //            word A = top->emu__DOT__console__DOT__adamnet__DOT__z80_addr;
                  //            if(PCBTable[A]) ReadPCB(A);
        //CData/*7:0*/ emu__DOT__console__DOT__adamnet__DOT__z80_data_rd;
                  //    }


#ifdef CPU_DEBUG
                        if (!top->reset) {
                                unsigned short pc = top->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__PC;

                                unsigned char di = top->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__di;
                                unsigned short ad = top->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__A;
                                unsigned char ir = top->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__IR;

                                unsigned char acc = top->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__ACC;
                                unsigned char z = top->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__flag_z;

                                unsigned char phi = top->emu__DOT__console__DOT__Cpu__DOT__cen;
                                unsigned char mcycle = top->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__mcycle;
                                unsigned char mreq = top->emu__DOT__console__DOT__Cpu__DOT__mreq_n;
                                bool ir_changed = top->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__ir_changed;

                                bool rom_read = top->emu__DOT__console__DOT__rom_read;
                                unsigned char E = top->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__i_reg__DOT__E;
                                unsigned char D = top->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__i_reg__DOT__D;

                                top->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__ir_changed = 0;

                                bool new_data = (mreq && !last_mreq && mcycle <= 4);
                                bool rom_data = (!rom_read && rom_read_last);
                                if ((rom_data) && !ir_changed) {
                                        std::string type = "NONE";
                                        if (new_data && !rom_data) { type = "NEW_ONLY"; }
                                        if (new_data && rom_data) { type = "BOTH_DATA"; }
                                        if (!new_data && rom_data) { type = "ROM_ONLY"; }

                                        std::string message = "%08d > ";
                                        message = message.append(type);
                                        message = message.append(" PC=%04x IR=%02x AD=%04x DI=%02x");

                                        //console.AddLog(message.c_str(), main_time, pc, ir, ad, di);


                                        ins_in[ins_index] = di;
                                        ins_index++;
                                        if (ins_index > ins_size - 1) { ins_index = 0; }
                                }

                                //console.AddLog("%08d PC=%04x IR=%02x AD=%04x DI=%02x ACC=%d Z=%d ND=%d IRC=%d", main_time, pc, ir, ad, di, acc, z, new_data, ir_changed, acc);

                                last_mreq = mreq;
                                rom_read_last = rom_read;

                                if (ir_changed) {
                                console.AddLog("%08d PC=%04x IR=%02x AD=%04x DI=%02x ACC=%x Z=%d ND=%d IRC=%d D=%x E=%x", main_time, pc, ir, ad, di, acc, z, new_data, ir_changed, D,E);

                                        //console.AddLog("%08d IR_CHANGED> PC=%04x IR=%02x AD=%04x DI=%02x ACC=%x z=%x", main_time, pc, ir, ad, di, acc, z);

                                        //console.AddLog("ACTIVE_IR: %x ACTIVE_PC: %x", active_ir, active_pc);

                                        if (active_ir_valid) {
                                                std::string opcode = get_opcode(active_ir, 0);

                                                // Is this a compound opcode?
                                                size_t pos = opcode.find("****");
                                                if (pos != std::string::npos)
                                                {
                                                        active_ir_ext = active_ir;
                                                }
                                                else {
                                                        unsigned char data1 = ins_in[ins_index - 2];
                                                        unsigned char data2 = ins_in[ins_index - 1];
                                                        data1 = ins_in[0];
                                                        data2 = ins_in[1];
                                                        std::string fmt = "%04X: ";
                                                        std::string opcode = get_opcode(active_ir, active_ir_ext);

                                                        size_t pos = opcode.find("&0000");
                                                        if (pos != std::string::npos)
                                                        {
                                                                //data1 = ins_in[0];
                                                                //data2 = ins_in[1];
                                                                char buf[6];
                                                                sprintf(buf, "$%02X%02X", data2, data1);
                                                                opcode.replace(pos, 5, buf);
                                                        }

                                                        pos = opcode.find("&4546");
                                                        if (pos != std::string::npos)
                                                        {
                                                                char buf[6];
                                                                char active_data = (ins_index == 1 ? data1 : data2);
                                                                unsigned short add = active_pc + +2;
                                                                if (opcode.substr(0, 4) == "djnz") {
                                                                        add = active_pc + ((signed char)active_data) + 2;
                                                                }
                                                                if (opcode.substr(0, 4) == "jr  ") {
                                                                        add = active_pc + ((signed char)active_data) + 2;
                                                                }
                                                                sprintf(buf, "$%04X", add);
                                                                opcode.replace(pos, 5, buf);
                                                        }

                                                        pos = opcode.find("&00");
                                                        if (pos != std::string::npos)
                                                        {
                                                                char buf[4];
                                                                sprintf(buf, "$%02X", ins_in[0]);
                                                                opcode.replace(pos, 3, buf);

                                                                pos = opcode.find("&00");
                                                                if (pos != std::string::npos)
                                                                {
                                                                        sprintf(buf, "$%02X", ins_in[1]);
                                                                        opcode.replace(pos, 3, buf);
                                                                }
                                                        }

                                                        fmt.append(opcode);
                                                        char buf[1024];
                                                        sprintf(buf, fmt.c_str(), active_pc);
                                                        writeLog(buf);

                                                        // Clear instruction cache
                                                        ins_index = 0;
                                                        for (int i = 0; i < ins_size; i++) {
                                                                ins_in[i] = 0;
                                                                ins_ma[i] = 0;
                                                        }
                                                        active_ir_ext = 0;
                                                        active_pc = ad;
                                                }
                                        }
                                        //console.AddLog("Setting active last_last_pc=%x last_pc=%x pc=%x addr=%x", last_last_pc, last_pc, pc, ad);
                                        active_ir_valid = true;
                                        ins_index = 0;
                                        active_ir = ir;

                                        last_last_pc = last_pc;
                                        last_pc = pc;
                                }
                        }
#endif
                        main_time++;
                }
                return 1;
        }

        // Stop verilating and cleanup
        top->final();
        delete top;
        exit(0);
        return 0;
}

unsigned char mouse_clock = 0;
unsigned char mouse_clock_reduce = 0;
unsigned char mouse_buttons = 0;
unsigned char mouse_x = 0;
unsigned char mouse_y = 0;

char spinner_toggle = 0;

int main(int argc, char** argv, char** env) {

        // Create core and initialise
        top = new Vemu();
        Verilated::commandArgs(argc, argv);
        Verilated::traceEverOn(true);
ROMPage[0] = (byte *)&top->emu__DOT__ram__DOT__ram;
ROMPage[1] = ROMPage[0]+0x2000;
ROMPage[2] = ROMPage[1]+0x2000;
ROMPage[3] = ROMPage[2]+0x2000;

ROMPage[4] = (byte *)&top->emu__DOT__upper_ram__DOT__ram;
ROMPage[5] = ROMPage[4]+0x2000;
ROMPage[6] = ROMPage[5]+0x2000;
ROMPage[7] = ROMPage[6]+0x2000;

RAMPage[0] = ROMPage[0];
RAMPage[1] = ROMPage[1];
RAMPage[2] = ROMPage[2];
RAMPage[3] = ROMPage[3];
RAMPage[4] = ROMPage[4];
RAMPage[5] = ROMPage[5];
RAMPage[6] = ROMPage[6];
RAMPage[7] = ROMPage[7];

LoadFDI(&Disks[0],"adam.dsk",FMT_ADMDSK);
LoadFDI(&Disks[4],"adam.ddp",FMT_DDP);


#ifdef WIN32
        // Attach debug console to the verilated code
        //Verilated::setDebug(console);
#endif

#ifdef CPU_DEBUG
        // Load debug opcodes
        loadOpcodes();

        // Load debug trace
        std::string line;
        std::ifstream fin(tracefilename);
        while (getline(fin, line)) {
                log_mame.push_back(line);
        }
#endif

        // Attach bus
        bus.ioctl_addr = &top->ioctl_addr;
        bus.ioctl_index = &top->ioctl_index;
        bus.ioctl_wait = &top->ioctl_wait;
        bus.ioctl_download = &top->ioctl_download;
        //bus.ioctl_upload = &top->ioctl_upload;
        bus.ioctl_wr = &top->ioctl_wr;
        bus.ioctl_dout = &top->ioctl_dout;
        //bus.ioctl_din = &top->ioctl_din;
        input.ps2_key = &top->ps2_key;

        // hookup blk device
        //blockdevice.MountDisk("adam.dsk",0);
        blockdevice.sd_lba[0] = &top->sd_lba[0];
        blockdevice.sd_lba[1] = &top->sd_lba[1];
        blockdevice.sd_lba[2] = &top->sd_lba[2];
        blockdevice.sd_lba[3] = &top->sd_lba[3];
        blockdevice.sd_lba[4] = &top->sd_lba[4];
        blockdevice.sd_lba[5] = &top->sd_lba[5];
        blockdevice.sd_lba[6] = &top->sd_lba[6];
        blockdevice.sd_lba[7] = &top->sd_lba[7];
        blockdevice.sd_rd = &top->sd_rd;
        blockdevice.sd_wr = &top->sd_wr;
        blockdevice.sd_ack = &top->sd_ack;
        blockdevice.sd_buff_addr= &top->sd_buff_addr;
        blockdevice.sd_buff_dout= &top->sd_buff_dout;
        blockdevice.sd_buff_din[0]= &top->sd_buff_din[0];
        blockdevice.sd_buff_din[1]= &top->sd_buff_din[1];
        blockdevice.sd_buff_wr= &top->sd_buff_wr;
        blockdevice.img_mounted= &top->img_mounted;
        blockdevice.img_readonly= &top->img_readonly;
        blockdevice.img_size= &top->img_size;


#ifndef DISABLE_AUDIO
        audio.Initialise();
#endif

        // Set up input module
        input.Initialise();
#ifdef WIN32
        input.SetMapping(input_up, DIK_UP);
        input.SetMapping(input_right, DIK_RIGHT);
        input.SetMapping(input_down, DIK_DOWN);
        input.SetMapping(input_left, DIK_LEFT);
        input.SetMapping(input_a, DIK_Z); // A
        input.SetMapping(input_b, DIK_X); // B
        input.SetMapping(input_x, DIK_A); // X
        input.SetMapping(input_y, DIK_S); // Y
        input.SetMapping(input_l, DIK_Q); // L
        input.SetMapping(input_r, DIK_W); // R
        input.SetMapping(input_select, DIK_1); // Select
        input.SetMapping(input_start, DIK_2); // Start
        input.SetMapping(input_menu, DIK_M); // System menu trigger

#else
        input.SetMapping(input_up, SDL_SCANCODE_UP);
        input.SetMapping(input_right, SDL_SCANCODE_RIGHT);
        input.SetMapping(input_down, SDL_SCANCODE_DOWN);
        input.SetMapping(input_left, SDL_SCANCODE_LEFT);
        input.SetMapping(input_a, SDL_SCANCODE_A);
        input.SetMapping(input_b, SDL_SCANCODE_B);
        input.SetMapping(input_x, SDL_SCANCODE_X);
        input.SetMapping(input_y, SDL_SCANCODE_Y);
        input.SetMapping(input_l, SDL_SCANCODE_L);
        input.SetMapping(input_r, SDL_SCANCODE_E);
        input.SetMapping(input_start, SDL_SCANCODE_1);
        input.SetMapping(input_select, SDL_SCANCODE_2);
        input.SetMapping(input_menu, SDL_SCANCODE_M);
#endif
        // Setup video output
        if (video.Initialise(windowTitle) == 1) { return 1; }

        //bus.QueueDownload("floppy.nib",1,0);
        //blockdevice.MountDisk("floppy.nib",0);
        //blockdevice.MountDisk("hd.hdv",1);
        blockdevice.MountDisk("adam.dsk",0);
        blockdevice.MountDisk("adam.ddp",4);

#ifdef WIN32
        MSG msg;
        ZeroMemory(&msg, sizeof(msg));
        while (msg.message != WM_QUIT)
        {
                if (PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE))
                {
                        TranslateMessage(&msg);
                        DispatchMessage(&msg);
                        continue;
                }
#else
        bool done = false;
        while (!done)
        {
                SDL_Event event;
                while (SDL_PollEvent(&event))
                {
                        ImGui_ImplSDL2_ProcessEvent(&event);
                        if (event.type == SDL_QUIT)
                                done = true;
                }
#endif
                video.StartFrame();

                input.Read();


                // Draw GUI
                // --------
                ImGui::NewFrame();

                // Simulation control window
                ImGui::Begin(windowTitle_Control);
                ImGui::SetWindowPos(windowTitle_Control, ImVec2(0, 0), ImGuiCond_Once);
                ImGui::SetWindowSize(windowTitle_Control, ImVec2(500, 150), ImGuiCond_Once);
                if (ImGui::Button("Reset simulation")) { resetSim(); } ImGui::SameLine();
                if (ImGui::Button("Start running")) { run_enable = 1; } ImGui::SameLine();
                if (ImGui::Button("Stop running")) { run_enable = 0; } ImGui::SameLine();
                ImGui::Checkbox("RUN", &run_enable);
                //ImGui::PopItemWidth();
                ImGui::SliderInt("Run batch size", &batchSize, 1, 250000);
                if (single_step == 1) { single_step = 0; }
                if (ImGui::Button("Single Step")) { run_enable = 0; single_step = 1; }
                ImGui::SameLine();
                if (multi_step == 1) { multi_step = 0; }
                if (ImGui::Button("Multi Step")) { run_enable = 0; multi_step = 1; }
                //ImGui::SameLine();
                ImGui::SliderInt("Multi step amount", &multi_step_amount, 8, 1024);
                ImGui::Checkbox("Adam", &adam_mode);
                ImGui::SameLine();
                if (ImGui::Button("Load ROM"))
    ImGuiFileDialog::Instance()->OpenDialog("ChooseFileDlgKey", "Choose File", ".rom", ".");

                if (ImGui::Button("PRINT MARKER")) { fprintf(stderr,"7F MARKER\n"); fprintf(stdout,"7F MARKER\n");  } ImGui::SameLine();
                //if (ImGui::Button("Soft Reset")) { fprintf(stderr,"soft reset\n"); soft_reset=1; } ImGui::SameLine();

                ImGui::End();

                // Debug log window
                console.Draw(windowTitle_DebugLog, &showDebugLog, ImVec2(500, 700));
                ImGui::SetWindowPos(windowTitle_DebugLog, ImVec2(0, 160), ImGuiCond_Once);

                // Memory debug
                ImGui::Begin("ram Editor");
                mem_edit.DrawContents(&top->emu__DOT__ram__DOT__ram , 32768, 0);
                ImGui::End();
                ImGui::Begin("upper ram Editor");
                mem_edit.DrawContents(&top->emu__DOT__upper_ram__DOT__ram, 32768, 0);
                ImGui::End();
                ImGui::Begin("lower expansion Editor");
                mem_edit.DrawContents(&top->emu__DOT__lowerexpansion_ram__DOT__mem, 32768, 0);
                ImGui::End();
                //ImGui::Begin("CHROM Editor");
                //mem_edit.DrawContents(top->emu__DOT__system__DOT__chrom__DOT__mem, 2048, 0);
                //ImGui::End();
                //ImGui::Begin("WKRAM Editor");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__wkram__DOT__mem, 16384, 0);
                //ImGui::End();
                //ImGui::Begin("CHRAM Editor");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__chram__DOT__mem, 2048, 0);
                //ImGui::End();
                //ImGui::Begin("FGCOLRAM Editor");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__fgcolram__DOT__mem, 2048, 0);
                //ImGui::End();
                //ImGui::Begin("BGCOLRAM Editor");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__bgcolram__DOT__mem, 2048, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite RAM");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__spriteram__DOT__mem, 96, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite Linebuffer RAM");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__spritelbram__DOT__mem, 1024, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite Collision Buffer RAM A");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__comet__DOT__spritecollisionbufferram_a__DOT__mem, 512, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite Collision Buffer RAM B");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__comet__DOT__spritecollisionbufferram_b__DOT__mem, 512, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite Collision RAM ");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__spritecollisionram__DOT__mem, 32, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite Debug RAM");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__spritedebugram__DOT__mem, 128000, 0);
                //ImGui::End();
                //ImGui::Begin("Palette ROM");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__palrom__DOT__mem, 64, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite ROM");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__spriterom__DOT__mem, 2048, 0);
                //ImGui::End();
                //ImGui::Begin("Tilemap ROM");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__tilemaprom__DOT__mem, 8192, 0);
                //ImGui::End();
                //ImGui::Begin("Tilemap RAM");
                //	mem_edit.DrawContents(&top->emu__DOT__system__DOT__tilemapram__DOT__mem, 768, 0);
                //ImGui::End();
                //ImGui::Begin("Sound ROM");
                //mem_edit.DrawContents(&top->emu__DOT__system__DOT__soundrom__DOT__mem, 64000, 0);
                //ImGui::End();

                int windowX = 550;
                int windowWidth = (VGA_WIDTH * VGA_SCALE_X) + 24;
                int windowHeight = (VGA_HEIGHT * VGA_SCALE_Y) + 90;

                // Video window
                ImGui::Begin(windowTitle_Video);
                ImGui::SetWindowPos(windowTitle_Video, ImVec2(windowX, 0), ImGuiCond_Once);
                ImGui::SetWindowSize(windowTitle_Video, ImVec2(windowWidth, windowHeight), ImGuiCond_Once);

                ImGui::SliderFloat("Zoom", &vga_scale, 1, 8); ImGui::SameLine();
                ImGui::SliderInt("Rotate", &video.output_rotate, -1, 1); ImGui::SameLine();
                ImGui::Checkbox("Flip V", &video.output_vflip);
                ImGui::Text("main_time: %ld frame_count: %d sim FPS: %f", main_time, video.count_frame, video.stats_fps);
                //ImGui::Text("pixel: %06d line: %03d", video.count_pixel, video.count_line);

                // Draw VGA output
                ImGui::Image(video.texture_id, ImVec2(video.output_width * VGA_SCALE_X, video.output_height * VGA_SCALE_Y));
                ImGui::End();

  if (ImGuiFileDialog::Instance()->Display("ChooseFileDlgKey"))
  {
    // action if OK
    if (ImGuiFileDialog::Instance()->IsOk())
    {
      std::string filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
      std::string filePath = ImGuiFileDialog::Instance()->GetCurrentPath();
      // action
fprintf(stderr,"filePathName: %s\n",filePathName.c_str());
fprintf(stderr,"filePath: %s\n",filePath.c_str());
     bus.QueueDownload(filePathName, 1,1);
    }

    // close
    ImGuiFileDialog::Instance()->Close();
  }


#ifndef DISABLE_AUDIO

                ImGui::Begin(windowTitle_Audio);
                ImGui::SetWindowPos(windowTitle_Audio, ImVec2(windowX, windowHeight), ImGuiCond_Once);
                ImGui::SetWindowSize(windowTitle_Audio, ImVec2(windowWidth, 250), ImGuiCond_Once);


                //float vol_l = ((signed short)(top->AUDIO_L) / 256.0f) / 256.0f;
                //float vol_r = ((signed short)(top->AUDIO_R) / 256.0f) / 256.0f;
                //ImGui::ProgressBar(vol_l + 0.5f, ImVec2(200, 16), 0); ImGui::SameLine();
                //ImGui::ProgressBar(vol_r + 0.5f, ImVec2(200, 16), 0);

                int ticksPerSec = (24000000 / 60);
                if (run_enable) {
                        audio.CollectDebug((signed short)top->AUDIO_L, (signed short)top->AUDIO_R);
                }
                int channelWidth = (windowWidth / 2)  -16;
                ImPlot::CreateContext();
                if (ImPlot::BeginPlot("Audio - L", ImVec2(channelWidth, 220), ImPlotFlags_NoLegend | ImPlotFlags_NoMenus | ImPlotFlags_NoTitle)) {
                        ImPlot::SetupAxes("T", "A", ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickMarks, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickMarks);
                        ImPlot::SetupAxesLimits(0, 1, -1, 1, ImPlotCond_Once);
                        ImPlot::PlotStairs("", audio.debug_positions, audio.debug_wave_l, audio.debug_max_samples, audio.debug_pos);
                        ImPlot::EndPlot();
                }
                ImGui::SameLine();
                if (ImPlot::BeginPlot("Audio - R", ImVec2(channelWidth, 220), ImPlotFlags_NoLegend | ImPlotFlags_NoMenus | ImPlotFlags_NoTitle)) {
                        ImPlot::SetupAxes("T", "A", ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickMarks, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickMarks);
                        ImPlot::SetupAxesLimits(0, 1, -1, 1, ImPlotCond_Once);
                        ImPlot::PlotStairs("", audio.debug_positions, audio.debug_wave_r, audio.debug_max_samples, audio.debug_pos);
                        ImPlot::EndPlot();
                }
                ImPlot::DestroyContext();
                ImGui::End();
#endif

                video.UpdateTexture();


                // Pass inputs to sim

                top->menu = input.inputs[input_menu];

                top->joystick_0 = 0;
                for (int i = 0; i < input.inputCount; i++)
                {
                        if (input.inputs[i]) { top->joystick_0 |= (1 << i); }
                }
                top->joystick_1 = top->joystick_0;

                /*top->joystick_analog_0 += 1;
                top->joystick_analog_0 -= 256;*/
                //top->paddle_0 += 1;
                //if (input.inputs[0] || input.inputs[1]) {
                //	spinner_toggle = !spinner_toggle;
                //	top->spinner_0 = (input.inputs[0]) ? 16 : -16;
                //	for (char b = 8; b < 16; b++) {
                //		top->spinner_0 &= ~(1UL << b);
                //	}
                //	if (spinner_toggle) { top->spinner_0 |= 1UL << 8; }
                //}

                mouse_buttons = 0;
                mouse_x = 0;
                mouse_y = 0;
                if (input.inputs[input_left]) { mouse_x = -2; }
                if (input.inputs[input_right]) { mouse_x = 2; }
                if (input.inputs[input_up]) { mouse_y = 2; }
                if (input.inputs[input_down]) { mouse_y = -2; }

                if (input.inputs[input_a]) { mouse_buttons |= (1UL << 0); }
                if (input.inputs[input_b]) { mouse_buttons |= (1UL << 1); }

                unsigned long mouse_temp = mouse_buttons;
                mouse_temp += (mouse_x << 8);
                mouse_temp += (mouse_y << 16);
                if (mouse_clock) { mouse_temp |= (1UL << 24); }
                mouse_clock = !mouse_clock;

                top->ps2_mouse = mouse_temp;
                top->ps2_mouse_ext = mouse_x + (mouse_buttons << 8);

                // Run simulation
                if (run_enable) {
                        for (int step = 0; step < batchSize; step++) { verilate(); }
                }
                else {
                        if (single_step) { verilate(); }
                        if (multi_step) {
                                for (int step = 0; step < multi_step_amount; step++) { verilate(); }
                        }
                }
        }

        // Clean up before exit
        // --------------------

#ifndef DISABLE_AUDIO
        audio.CleanUp();
#endif
        video.CleanUp();
        input.CleanUp();

        return 0;
}
