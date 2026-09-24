#include <verilated.h>
//#include "verilated_fst_sc.h"
#include "Vemu.h"
#include "Vemu__Syms.h"

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

#define VERILATOR_MAJOR_VERSION (VERILATOR_VERSION_INTEGER / 1000000)

#if VERILATOR_MAJOR_VERSION >= 5
#define VERTOPINTERN top->rootp
#else
#define VERTOPINTERN top
#endif

#include "sim_console.h"
#include "sim_bus.h"
#include "sim_blkdevice.h"
#include "sim_video.h"
#include "sim_audio.h"
#include "sim_input.h"
#include "sim_clock.h"
#include "sim_adam_keys.h"

#include "../imgui/imgui_memory_editor.h"
#include "../imgui/ImGuiFileDialog.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <iterator>
#include <string>
#include <sys/stat.h>
#include <iomanip>
#include <vector>
#include <chrono>
#include <algorithm>
using namespace std;

// Simulation control
// ------------------
int initialReset = 48;
bool run_enable = 1;
bool adam_mode= 1;
// Memory expander for sim.v exp_ram, in the OSD's order:
// 0 = 64K, 1 = 256K, 2 = 512K, 3 = 1M, 4 = 2M, 5 = none. Banks past 64K are
// selected by port 42h.
int exp_ram_mode = 0;
int spin_mode_opt = 0;  // sim.v spin_mode: 0 = off, 1 = spinner device (set by --spin)
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
// Indices 0-19 are joystick bits, in the order of the core's "J," CONF_STR entry, so that the
// loop that builds joystick_0 can just shift by the index. Index 20 is the system menu, which is
// not a joystick bit and is excluded from that loop.
SimInput input(30, console);
const int input_right = 0;
const int input_left = 1;
const int input_down = 2;
const int input_up = 3;
const int input_fire1 = 4;
const int input_fire2 = 5;
const int input_star = 6;
const int input_pound = 7;
const int input_kp0 = 8;        // keypad 0-9 run from here to index 17
const int input_purple = 18;
const int input_blue = 19;
const int input_menu = 20;
const int input_joy_bits = 20;  // how many of the above are joystick bits
// Capture hotkeys. Function keys, so they cannot collide with anything the ColecoVision
// controller needs.
const int input_cap_start = 21;  // [
const int input_cap_stop  = 22;  // ]
const int input_cap_one   = 23;  // backslash
// Player 2. Only the directions and the two fire buttons: the keypad is on player 1's number
// row and that is enough to get through a game's menus.
const int input_p2_right = 24;
const int input_p2_left  = 25;
const int input_p2_down  = 26;
const int input_p2_up    = 27;
const int input_p2_fire1 = 28;
const int input_p2_fire2 = 29;
// Kept for the mouse code below, which wants a couple of buttons by their old names.
const int input_a = input_fire1;
const int input_b = input_fire2;

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
Byte *ROMPage[8];              /* 8x8kB read-only (ROM) pages   */
Byte *RAMPage[8];              /* 8x8kB read-write (RAM) pages  */
Byte Port60=1;                   /* Adam port 0x60-0x7F (memory)  */

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
        unsigned char in1 = VERTOPINTERN->emu__DOT__system__DOT__in_p1_data;
        sprintf(buf, " %02X", in1);
        c.append(buf);
        unsigned char in2 = VERTOPINTERN->emu__DOT__system__DOT__in_p2_data;
        sprintf(buf, " %02X", in2);
        c.append(buf);
        unsigned char in3 = VERTOPINTERN->emu__DOT__system__DOT__in_p3_data;
        sprintf(buf, " %02X", in3);
        c.append(buf);
        unsigned char in4 = VERTOPINTERN->emu__DOT__system__DOT__in_p4_data;
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
        VERTOPINTERN->reset = 1;
        clk_sys.Reset();
}

int verilate() {

        if (!Verilated::gotFinish()) {
                if (soft_reset){
                        fprintf(stderr,"soft_reset.. in gotFinish\n");
                        VERTOPINTERN->soft_reset = 1;
                        soft_reset=0;
                        soft_reset_time=0;
                        fprintf(stderr,"turning on %x\n",VERTOPINTERN->soft_reset);
                }
                if (clk_sys.IsRising()) {
                        soft_reset_time++;
                }
                if (soft_reset_time==initialReset) {
                        VERTOPINTERN->soft_reset = 0;
                        fprintf(stderr,"turning off %x\n",VERTOPINTERN->soft_reset);
                        fprintf(stderr,"soft_reset_time %ld initialReset %x\n",soft_reset_time,initialReset);
                }

                // Hold reset while a ROM downloads, like ColecoAdam.sv does
                static bool was_downloading = false;
                bool downloading = bus.HasQueue() || *bus.ioctl_download;
                // Assert reset during startup
                if (main_time < initialReset || downloading) { VERTOPINTERN->reset = 1; }
                // Deassert reset after startup, or when a download finishes
                if (main_time >= initialReset && !downloading && (main_time == initialReset || was_downloading)) { VERTOPINTERN->reset = 0; }
                was_downloading = downloading;

                // Clock dividers
                clk_sys.Tick();

                // Set system clock in core
                VERTOPINTERN->clk_sys = clk_sys.clk;
                VERTOPINTERN->adam = adam_mode;
                VERTOPINTERN->exp_ram = exp_ram_mode;
                // --spin drives the spinner device path, like the OSD's "Spinner" setting
                VERTOPINTERN->spin_mode = spin_mode_opt;

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
                        audio.Clock(VERTOPINTERN->AUDIO_L, VERTOPINTERN->AUDIO_R);
                }
#endif

                // Output pixels on rising edge of pixel clock
                if (clk_sys.IsRising() && VERTOPINTERN->CE_PIXEL ) {
                        uint32_t colour = 0xFF000000 | VERTOPINTERN->VGA_B << 16 | VERTOPINTERN->VGA_G << 8 | VERTOPINTERN->VGA_R;
                        video.Clock(VERTOPINTERN->VGA_HB, VERTOPINTERN->VGA_VB, VERTOPINTERN->VGA_HS, VERTOPINTERN->VGA_VS, colour);
                }

                if (clk_sys.IsRising()) {


                  /*A
                        /* ADAM NET */
                  //                        if (VERTOPINTERN->emu__DOT__console__DOT__adamnet__DOT__adam_reset_pcb_n_i == 0) // negative signal
                  //     {
                  //             printf("ResetPCB from sim_main\n");
                  //            ResetPCB();
                  //    }
                  //    /* if we are writing -- check it ? */
                  //    if (VERTOPINTERN->emu__DOT__console__DOT__adamnet__DOT__z80_wr)
                  //    {
                  //
                  //            word A = VERTOPINTERN->emu__DOT__console__DOT__adamnet__DOT__z80_addr;
                  //            word V = VERTOPINTERN->emu__DOT__console__DOT__adamnet__DOT__z80_data_wr;
                  //            if(PCBTable[A]) {
                  //                    printf("z80_wr, WritePCB A %x V %x %x\n",A,V,PCBTable[A]);
                  //                    WritePCB(A,V);
                  //            }
                  //    }
                  //    if (VERTOPINTERN->emu__DOT__console__DOT__adamnet__DOT__z80_rd)
                  //    {
                  //            word A = VERTOPINTERN->emu__DOT__console__DOT__adamnet__DOT__z80_addr;
                  //            if(PCBTable[A]) ReadPCB(A);
        //CData/*7:0*/ emu__DOT__console__DOT__adamnet__DOT__z80_data_rd;
                  //    }


#ifdef CPU_DEBUG
                        if (!VERTOPINTERN->reset) {
                                unsigned short pc = VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__PC;

                                unsigned char di = VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__di;
                                unsigned short ad = VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__A;
                                unsigned char ir = VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__IR;

                                unsigned char acc = VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__ACC;
                                unsigned char z = VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__flag_z;

                                unsigned char phi = VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__cen;
                                unsigned char mcycle = VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__mcycle;
                                unsigned char mreq = VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__mreq_n;
                                bool ir_changed = VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__ir_changed;

                                bool rom_read = VERTOPINTERN->emu__DOT__console__DOT__rom_read;
                                unsigned char E = VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__i_reg__DOT__E;
                                unsigned char D = VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__i_reg__DOT__D;

                                VERTOPINTERN->emu__DOT__console__DOT__Cpu__DOT__i_tv80_core__DOT__ir_changed = 0;

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

// Command line options
// --------------------
bool headless = false;          // --headless: no window, exit after --frames
int run_frames = 0;             // --frames N: exit after N video frames (0 = never)
std::string cart_file;          // --cart FILE
std::string shot_dir = ".";     // --outdir DIR
std::vector<int> shot_frames;   // --shots F1,F2,...
int shot_every = 0;             // --every K
bool exit_requested = false;

struct KeyPress { int bit; int frame; int length; };
std::vector<KeyPress> key_presses;      // --press KEY@FRAME[:LEN]

// Adam media and keyboard
std::string disk_file[4], tape_file[4];         // --disk N FILE, --tape N FILE
const int kKeyGap = 8;                          // frames per typed key (colem_ref uses the same)
const int kKeyHold = 4;                         // frames a typed key stays down
struct PS2Event { int frame; int code; bool pressed; };
std::vector<PS2Event> ps2_events;               // --type TEXT@FRAME, --key NAME@FRAME
size_t ps2_next = 0;

// joystick_0 bit for a key name, in the order of the core's "J," CONF_STR entry
int keyBit(const std::string& key) {
        static const char* names[] = { "right", "left", "down", "up", "fire1", "fire2", "star", "pound",
                                       "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "purple", "blue" };
        for (int i = 0; i < 20; i++) {
                if (key == names[i]) { return i; }
        }
        return -1;
}

uint32_t scriptedJoystick(int frame) {
        uint32_t bits = 0;
        for (const KeyPress& p : key_presses) {
                if (frame >= p.frame && frame < p.frame + p.length) { bits |= 1UL << p.bit; }
        }
        return bits;
}

// --record FILE / --replay FILE: play a game in the GUI, then run the same session again headless
// with probes on. Some faults only show up somewhere a scripted --press sequence cannot reach -
// several rooms into a game, say - and this is the way to get there and then study it repeatably.
//
// The file is one line per change, "frame joystick_bits_in_hex", so it is small, readable, and can
// be trimmed or hand-edited to isolate a moment. Replay is faithful because the core has no
// randomness: the same input on the same frames gives the same run. The one imprecision is that a
// press which happened part way through a frame is replayed from that frame's start.
struct InputFrame { int frame; uint32_t p1, p2; };
static FILE* record_fp = nullptr;
static uint32_t record_prev1 = 0, record_prev2 = 0;
static bool record_started = false;
static std::vector<InputFrame> replay_events;
static size_t replay_next = 0;
static uint32_t replay_p1 = 0, replay_p2 = 0;

// F5 / F6 / F7 in the GUI: start capturing, stop capturing, capture this one frame.
//
// Each capture writes the picture and, beside it, the VDP state that produced it - the eight
// control registers and the whole 16K of VRAM. That second file is the useful one: with it the
// background can be re-rendered in software (verilator/compare/tools/vdpref.py) and compared
// against what the core actually drew, which is what separates "the VDP drew this wrongly" from
// "the game put this in VRAM". Without it a screenshot of a glitch is only a picture of a glitch.
static std::string capture_dir = "captures";
static int capture_from = -1, capture_to = -1;

// While capturing, every CPU write that reaches VRAM is logged with the scanline it landed on.
// That is what an end-of-frame VRAM snapshot cannot tell you: whether the program changed a
// table while the beam was already past it. A frame that looks torn is then explained without
// guessing - either the writes are in the vertical blank, and the core drew a static table
// wrongly, or they are in the active display, and the tear is the program's own doing.
//
// The scanline is the VDP's vertical counter, which is negative in the top border, 0..191 in
// the active display and 192+ at the bottom, so "was this in blanking" reads straight off it.
static FILE* write_log = nullptr;
static int wl_abort_prev = 0;
static unsigned long wl_lines = 0;
static int wl_frame_prev = -1;

void logVramWrite(int frame) {
        if (!write_log) { return; }
        int abort = VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__abort_wrvram_s;
        if (abort && !wl_abort_prev) {
                int addr = VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__addr_q & 0x3FFF;
                int data = VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__buffer_q;
                int line = (int16_t)(VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__hor_vert_b__DOT__cnt_vert_q << 7) >> 7;
                fprintf(write_log, "%d %d %04X %02X\n", frame, line, addr, data);
                wl_lines++;
        }
        wl_abort_prev = abort;
}

// Eight control register bytes, then the whole 16K of VRAM. vdpref.py reads exactly this.
void saveVdpState(const std::string& path) {
        FILE* f = fopen(path.c_str(), "wb");
        if (!f) { fprintf(stderr, "cannot write %s\n", path.c_str()); return; }
        for (int r = 0; r < 8; r++) {
                unsigned char v = VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__ctrl_reg_q[r];
                fwrite(&v, 1, 1, f);
        }
        for (int a = 0; a < 16384; a++) {
                unsigned char v = VERTOPINTERN->emu__DOT__vram__DOT__mem[a];
                fwrite(&v, 1, 1, f);
        }
        fclose(f);
}
static bool capturing = false;
static int captured = 0;

void captureFrame(int frame, const char* why) {
        mkdir(capture_dir.c_str(), 0755);
        char name[64];
        snprintf(name, sizeof(name), "/cap_%05d.ppm", frame);
        std::string ppm = capture_dir + name;
        if (!video.SavePPM(ppm.c_str())) {
                fprintf(stderr, "cannot write %s\n", ppm.c_str());
                return;
        }
        snprintf(name, sizeof(name), "/cap_%05d.vdp", frame);
        saveVdpState(capture_dir + name);
        captured++;
        printf("capture %s: frame %d -> %s (+.vdp)\n", why, frame, ppm.c_str());
        fflush(stdout);
}

void captureStart(int frame) {
        capturing = true;
        mkdir(capture_dir.c_str(), 0755);
        std::string wl = capture_dir + "/writes.txt";
        write_log = fopen(wl.c_str(), "w");
        if (write_log) { fprintf(write_log, "# frame scanline vram_addr data\n"); }
        printf("capture: started at frame %d, writing to %s/\n", frame, capture_dir.c_str());
        fflush(stdout);
}

void captureStop(int frame) {
        capturing = false;
        if (write_log) { fclose(write_log); write_log = nullptr; }
        printf("capture: stopped at frame %d, %d frames and %lu vram writes logged\n",
               frame, captured, wl_lines);
        fflush(stdout);
}

void captureHotkeys(int frame) {
        static bool prev_start = false, prev_stop = false, prev_one = false;
        bool start = input.inputs[input_cap_start];
        bool stop  = input.inputs[input_cap_stop];
        bool one   = input.inputs[input_cap_one];

        if (start && !prev_start && !capturing) { captureStart(frame); }
        if (stop && !prev_stop && capturing) { captureStop(frame); }
        if (one && !prev_one) { captureFrame(frame, "single"); }
        if (capturing) { captureFrame(frame, "run"); }

        prev_start = start; prev_stop = stop; prev_one = one;
}

void recordJoystick(int frame, uint32_t p1, uint32_t p2) {
        if (!record_fp) { return; }
        if (record_started && p1 == record_prev1 && p2 == record_prev2) { return; }
        fprintf(record_fp, "%d %X %X\n", frame, p1, p2);
        fflush(record_fp);
        record_prev1 = p1;
        record_prev2 = p2;
        record_started = true;
}

void replayJoystick(int frame, uint32_t* p1, uint32_t* p2) {
        while (replay_next < replay_events.size() && replay_events[replay_next].frame <= frame) {
                replay_p1 = replay_events[replay_next].p1;
                replay_p2 = replay_events[replay_next].p2;
                replay_next++;
        }
        *p1 = replay_p1;
        *p2 = replay_p2;
}

void loadReplay(const std::string& path) {
        FILE* f = fopen(path.c_str(), "r");
        if (!f) { fprintf(stderr, "cannot read %s\n", path.c_str()); exit(1); }
        char line[128];
        int one_column = 0;
        while (fgets(line, sizeof(line), f)) {
                int frame; unsigned p1, p2;
                int n = sscanf(line, "%d %x %x", &frame, &p1, &p2);
                if (n == 3) {
                        replay_events.push_back({frame, p1, p2});
                } else if (n == 2) {
                        // Recorded before player 2 had its own keys, when the simulator fed one
                        // value to both ports. Reproduce that, or the recording stops replaying
                        // the session it captured.
                        replay_events.push_back({frame, p1, p1});
                        one_column++;
                }
        }
        fclose(f);
        printf("replay: %zu input changes, last at frame %d%s\n", replay_events.size(),
               replay_events.empty() ? -1 : replay_events.back().frame,
               one_column ? "  (pre-2026-09-19 format: both ports driven together)" : "");
}

// --spin/--spin2 STEPS@FRAME[:LEN]: turn the roller STEPS notches per frame for LEN frames.
// This drives the core the way a MiSTer spinner device does: a signed step count with bit 8
// toggled on every update.
struct SpinSpec { int port; int frame; int steps; int length; };
std::vector<SpinSpec> spins;
unsigned char spin_toggle[2] = { 0, 0 };

// --peek [v:]ADDR[:LEN]@FRAME prints bytes of memory when FRAME is reached. Without a prefix it
// reads by Z80 address: below 8000h the console's lower RAM array, at 8000h and above the ADAM's
// upper 32K, which is where EOS keeps its variables. Console mode mirrors its 1K, so Z80 7038h is
// index 6038h, and has a cartridge rather than RAM above 8000h. With "v:" it reads the 16K of
// VRAM instead, which is where the VDP tables live.
struct PeekSpec { int addr; int len; int frame; bool vram; };
std::vector<PeekSpec> peeks;

static unsigned char peekByte(const PeekSpec& p, int i) {
        int a = p.addr + i;
        if (p.vram) { return VERTOPINTERN->emu__DOT__vram__DOT__mem[a & 0x3FFF]; }
        if (a & 0x8000) { return VERTOPINTERN->emu__DOT__upper_ram__DOT__ram[a & 0x7FFF]; }
        return VERTOPINTERN->emu__DOT__ram__DOT__ram[a & 0x7FFF];
}

void scriptedPeek(int frame) {
        for (const PeekSpec& p : peeks) {
                if (p.frame != frame) { continue; }
                printf("peek frame %d %s%04X:", frame, p.vram ? "vram " : "", p.addr);
                for (int i = 0; i < p.len; i++) { printf(" %02X", peekByte(p, i)); }
                printf("\n");
                fflush(stdout);
        }
}

void scriptedSpinner(int frame) {
        for (int port = 0; port < 2; port++) {
                int steps = 0;
                for (const SpinSpec& s : spins) {
                        if (s.port == port && frame >= s.frame && frame < s.frame + s.length) { steps += s.steps; }
                }
                if (steps == 0) { continue; }
                if (steps > 127) { steps = 127; }
                if (steps < -127) { steps = -127; }
                spin_toggle[port] ^= 1;
                uint32_t v = ((uint32_t)spin_toggle[port] << 8) | (uint8_t)steps;
                if (port == 0) { VERTOPINTERN->spinner_0 = v; } else { VERTOPINTERN->spinner_1 = v; }
        }
}

// Adam key names; the same list as colem_ref's --key
int adamKeyName(const std::string& name) {
        static const struct { const char* name; int code; } keys[] = {
                { "enter", 0x0D }, { "esc", 0x1B }, { "bs", 0x08 }, { "tab", 0x09 }, { "space", 0x20 },
                { "up", 0xA0 }, { "right", 0xA1 }, { "down", 0xA2 }, { "left", 0xA3 }, { "home", 0x80 },
                { "f1", 0x81 }, { "f2", 0x82 }, { "f3", 0x83 }, { "f4", 0x84 }, { "f5", 0x85 }, { "f6", 0x86 },
                { "wildcard", 0x90 }, { "undo", 0x91 }, { "move", 0x92 }, { "store", 0x93 },
                { "insert", 0x94 }, { "print", 0x95 }, { "clear", 0x96 }, { "delete", 0x97 },
        };
        for (const auto& k : keys) {
                if (name == k.name) { return k.code; }
        }
        return -1;
}

// Queue the PS/2 make/break events that type one Adam key code at a frame
bool queueAdamKey(int code, int frame) {
        int ps2 = adam_ps2_keys[code & 0xFF];
        if (!ps2) { return false; }
        int key = ps2 & 0x1FF;
        bool shift = ps2 & 0x200;
        if (shift) { ps2_events.push_back({ frame, 0x12, true }); }
        ps2_events.push_back({ frame, key, true });
        ps2_events.push_back({ frame + kKeyHold, key, false });
        if (shift) { ps2_events.push_back({ frame + kKeyHold, 0x12, false }); }
        return true;
}

// Present queued PS/2 events on ps2_key, spaced out so the core sees every toggle of bit 10
void feedPS2() {
        static int gap = 0;
        static bool toggle = false;
        if (gap > 0) { gap--; return; }
        if (ps2_next >= ps2_events.size() || ps2_events[ps2_next].frame > video.count_frame) { return; }
        const PS2Event& e = ps2_events[ps2_next++];
        toggle = !toggle;
        VERTOPINTERN->ps2_key = e.code | (e.pressed << 9) | (toggle << 10);
        gap = 20000;
}

// Run one verilate() step, then apply --press, --shots and --frames when a new video frame starts
int last_frame = 0;
// SIM_ADDR_PROFILE=N samples the Z80 address bus every N steps and prints the busiest addresses at
// the end of a headless run: a cheap way to see where a program is looping, and on what.
static int addr_profile_every = 0;
static long addr_profile_step = 0;
static std::vector<unsigned long> addr_hist;
// SIM_ADDR_PROFILE_FROM=N ignores everything before frame N, so one phase of a program can be
// profiled on its own rather than averaged with its start-up.
static int addr_profile_from = 0;
// SIM_MEGA_TRACE=1 prints every MegaCart bank switch with the frame it happened on.
static int mega_trace = 0;
// SIM_VDP_TRACE=1 prints every VDP control register write: register 1 says whether the display is
// on, register 5 where the sprite attribute table lives.
static int vdp_trace = 0, vdp_wr_prev = 0;

// SIM_SPR5_PROFILE=1 watches the VDP's fifth-sprite status: how often the sprite engine reports
// a fifth sprite and with which number, and what the CPU would read back in the low 5 bits of the
// status register. Games use that number as a scanline counter, so "never set" or "always the
// same number" is the signature of a broken one.
static int spr5_profile = 0;
static unsigned long spr5_events = 0;
static std::vector<unsigned long> spr5_detect_hist;   // numbers the sprite engine reported
static std::vector<int> spr5_line_min, spr5_line_max; // and the scanlines they were reported on
static std::vector<unsigned long> spr5_latched_hist;  // numbers visible in the status register
static unsigned long spr5_flag_steps = 0, spr5_steps = 0;
// and the VDP's interrupt output, which the ColecoVision wires to the Z80's NMI: a program that
// sits in HALT is waiting for this, so "stopped falling" explains a frozen screen.
static unsigned long vdp_int_falls = 0;
static int vdp_int_last_frame = -1, vdp_int_prev = 1;
// A read of the status register is supposed to clear the interrupt flag. vdp18_cpuio writes it as
//   if (irq_i) int_n_q <= 0; else if (destr_rd_status_s) int_n_q <= 1;
// so a read that lands in the same cycle the VDP is setting the flag loses its clear and the flag
// stays set. That is the hazard the F18A changelog calls out as a ColecoVision problem, fixed
// there by always letting the read win. Counting reads that should have cleared the flag against
// the times the flag actually rose says how often it bites: a game polling the status register in
// a tight vblank loop that sees a stale flag will run its next update during the active display.
static unsigned long vdp_int_rises = 0, vdp_clearing_reads = 0;
static int vdp_destr_prev = 0;
// and the MegaCart bank, since a cartridge that stops paging has stopped loading
// A CPU write to VRAM is held until the VDP can fit it into an access slot. If the program
// writes again before that happens the earlier byte never reaches VRAM, exactly as on the real
// chip - so a high collision count is a program outrunning the VDP, and shows up as tiles that
// never change.
static unsigned long vram_write_reqs = 0, vram_write_collisions = 0;
static int vram_sched_prev = 0;
// How long each of those writes waited for its slot, in 372 ns memory cycles. The datasheet
// (2.1.6) allows the VDP up to 16 of them in Graphics mode with sprites - "CPU windows occur
// once every 16 memory cycles giving a maximum delay of 6 microseconds" - so a max of 16 here
// means the core is reproducing the real access window rather than being more generous than the
// chip. Buckets: <=1, 2-3, 4-7, 8-15, 16+.
static unsigned long vram_wait_hist[5] = {0, 0, 0, 0, 0};
static unsigned long vram_wait_max = 0, vram_wait = 0;
// abort_wrvram_s is combinational and stays up for the whole access slot, and this loop samples
// faster than that, so both it and the slot enable need edge detection or every write is counted
// more than once.
static int vram_acc_prev = 0, vram_abort_prev = 0;
static unsigned long mega_switches = 0;
static int mega_last_frame = -1, mega_prev = -1;
static std::vector<unsigned long> mega_hist;

void stepSim() {
        feedPS2();
        verilate();
        if (addr_profile_every > 0 && video.count_frame >= addr_profile_from
            && ++addr_profile_step % addr_profile_every == 0) {
                addr_hist[VERTOPINTERN->emu__DOT__console__DOT__adamnet__DOT__z80_addr]++;
        }
        if (spr5_profile && video.count_frame >= addr_profile_from) {
                spr5_steps++;
                if (VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__spr_5th_s) {
                        int num = VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__spr_5th_num_s & 31;
                        spr5_events++;
                        spr5_detect_hist[num]++;
                        int line = (int16_t)(VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__hor_vert_b__DOT__cnt_vert_q << 7) >> 7;
                        if (line < spr5_line_min[num]) spr5_line_min[num] = line;
                        if (line > spr5_line_max[num]) spr5_line_max[num] = line;
                }
                if (VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__sprite_5th_q) {
                        spr5_flag_steps++;
                        spr5_latched_hist[VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__sprite_5th_num_q & 31]++;
                }
        }
        if (vdp_trace) {
                        int wr = VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__write_reg_s;
                        if (wr && !vdp_wr_prev) {
                                int reg = VERTOPINTERN->emu__DOT__console__DOT__d_from_cpu_s & 7;
                                int val = VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__tmp_q;
                                printf("vdp frame %d: R%d = %02X%s\n", video.count_frame, reg, val,
                                       reg == 1 ? ((val & 0x40) ? "  (display on)" : "  (display OFF)") : "");
                                fflush(stdout);
                        }
                        vdp_wr_prev = wr;
        }
        if (spr5_profile && video.count_frame >= addr_profile_from) {

                int sched = VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__wrvram_sched_q;
                if (sched && !vram_sched_prev) {
                        vram_write_reqs++;
                        if (VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__wrvram_q) {
                                vram_write_collisions++;
                        }
                }
                vram_sched_prev = sched;

                int acc = VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__clk_en_acc_s;
                int pending = sched || VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__wrvram_q;
                if (acc && !vram_acc_prev && pending) { vram_wait++; }
                vram_acc_prev = acc;
                int abort = VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__abort_wrvram_s;
                if (abort && !vram_abort_prev) {
                        // The slot that services the write is counted above too, so take it back
                        // off: the figure wanted is how many slots went by before that one, which
                        // is what the datasheet's "once every 16 memory cycles" is measuring.
                        unsigned long w = vram_wait ? vram_wait - 1 : 0;
                        if (w > vram_wait_max) { vram_wait_max = w; }
                        vram_wait_hist[w <= 1 ? 0 : w <= 3 ? 1 : w <= 7 ? 2 : w <= 15 ? 3 : 4]++;
                        vram_wait = 0;
                }
                vram_abort_prev = abort;

                int vint = VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__int_n_q;
                if (vdp_int_prev && !vint) { vdp_int_falls++; vdp_int_last_frame = video.count_frame; }
                if (!vdp_int_prev && vint) { vdp_int_rises++; }
                int destr = VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__cpu_io_b__DOT__destr_rd_status_s;
                if (destr && !vdp_destr_prev && !vint) { vdp_clearing_reads++; }
                vdp_destr_prev = destr;
                vdp_int_prev = vint;

                int page = VERTOPINTERN->emu__DOT__console__DOT__addr_dec_b__DOT__megacart_page & 63;
                if (page != mega_prev) {
                        mega_switches++;
                        mega_last_frame = video.count_frame;
                        if (mega_trace) { printf("megacart frame %d: page %d\n", video.count_frame, page); fflush(stdout); }
                        mega_prev = page;
                }
                mega_hist[page]++;
        }
        // Per step, not per frame: a torn frame is torn because of when inside it the write
        // happened.
        if (write_log) { logVramWrite(video.count_frame); }
        // Mark where the frame counter turns over, with the scanline it happens on. The
        // picture saved as frame N is the one drawn *before* that mark, so without this the
        // writes and the frames they belong to can be attributed a frame apart.
        if (write_log && video.count_frame != wl_frame_prev) {
                int line = (int16_t)(VERTOPINTERN->emu__DOT__console__DOT__vdp18_b__DOT__hor_vert_b__DOT__cnt_vert_q << 7) >> 7;
                fprintf(write_log, "# frame counter -> %d at scanline %d\n", video.count_frame, line);
                wl_frame_prev = video.count_frame;
        }

        if (video.count_frame == last_frame) { return; }
        last_frame = video.count_frame;

        if (headless) {
                uint32_t rp1 = 0, rp2 = 0;
                replayJoystick(last_frame, &rp1, &rp2);
                VERTOPINTERN->joystick_0 = scriptedJoystick(last_frame) | rp1;
                VERTOPINTERN->joystick_1 = rp2;
                // Recording a headless run too is what lets --press and --replay be checked
                // against each other: record a scripted run, replay it, compare the frames.
                recordJoystick(last_frame, VERTOPINTERN->joystick_0, VERTOPINTERN->joystick_1);
        }
        if (!spins.empty()) { scriptedSpinner(last_frame); }
        if (!peeks.empty()) { scriptedPeek(last_frame); }

        bool shot = shot_every > 0 && last_frame % shot_every == 0;
        for (int f : shot_frames) {
                if (f == last_frame) { shot = true; }
        }
        if (shot) {
                char name[32];
                snprintf(name, sizeof(name), "/frame_%05d.ppm", last_frame);
                std::string path = shot_dir + name;
                if (!video.SavePPM(path.c_str())) { fprintf(stderr, "Cannot write %s\n", path.c_str()); }
                // The VDP state beside every saved frame, so a frame can always be checked
                // against what its own tables say it should be without running the game again.
                snprintf(name, sizeof(name), "/frame_%05d.vdp", last_frame);
                saveVdpState(shot_dir + name);
        }

        if (!headless) { captureHotkeys(last_frame); }
        // --capture-frames A-B does the same thing without the GUI, so a session that was
        // recorded once can be re-captured with different probes.
        if (capture_from >= 0) {
                if (last_frame == capture_from && !capturing) { captureStart(last_frame); }
                if (capturing) { captureFrame(last_frame, "range"); }
                if (last_frame == capture_to && capturing) { captureStop(last_frame); }
        }

        if (run_frames > 0 && last_frame >= run_frames) { exit_requested = true; }
}

void usage(const char* prog) {
        fprintf(stderr,
                "Usage: %s [options]\n"
                "  --cart FILE            load a cartridge (.col/.rom/.bin)\n"
                "  --console              ColecoVision console mode\n"
                "  --adam                 Adam computer mode (default)\n"
                "  --exp-ram SIZE         memory expander: 64 (default), 256, 512, 1024, 2048 or none.\n"
                "                         Past 64K the bank is chosen by port 42h; 64 has no bank register\n"
                "  --headless             run without a window; requires --frames\n"
                "  --record FILE         write controller input to FILE while you play in the GUI\n"
                "  --replay FILE         drive the controller from a FILE written by --record\n"
                "  --capture-dir DIR     where F5/F6/F7 write captures (default ./captures)\n"
                "                        F5 start capturing, F6 stop, F7 grab this frame;\n"
                "                        each writes cap_NNNNN.ppm and cap_NNNNN.vdp (regs+VRAM)\n"
                "  --frames N             exit after N video frames\n"
                "  --shots F1,F2,...      save these frames as DIR/frame_NNNNN.ppm\n"
                "  --every K              save every Kth frame\n"
                "  --outdir DIR           where to save frames (default .)\n"
                "  --press KEY@FRAME[:N]  hold KEY on controller 1 for N frames (default 8)\n"
                "                         KEY: 0-9 star pound up down left right fire1 fire2 purple blue\n"
                "  --spin STEPS@FRAME[:N] turn controller 1's roller STEPS notches per frame for N frames\n"
                "                         (negative spins the other way); --spin2 for controller 2\n"
                "  --peek [v:]ADDR[:N]@FRAME  print N bytes of RAM, or of VRAM with v:, at FRAME\n"
                "  --disk N FILE          mount a floppy image in drive N (1-4); writes go back to FILE\n"
                "  --tape N FILE          mount a tape image in drive N (1-4); writes go back to FILE\n"
                "                         (without --disk/--tape, adam.dsk and adam.ddp are mounted if present)\n"
                "  --type TEXT@FRAME      type TEXT on the Adam keyboard from FRAME, 8 frames per key (\\n = Return)\n"
                "  --key NAME@FRAME       press one Adam key: enter esc bs tab space up down left right home\n"
                "                         f1-f6 undo wildcard move store insert print clear delete\n",
                prog);
}

void parseArgs(int argc, char** argv) {
        for (int i = 1; i < argc; i++) {
                std::string arg = argv[i];
                auto value = [&]() -> std::string {
                        if (i + 1 >= argc) { fprintf(stderr, "%s needs a value\n", arg.c_str()); exit(1); }
                        return argv[++i];
                };

                if (arg == "--help" || arg == "-h") { usage(argv[0]); exit(0); }
                else if (arg == "--cart") { cart_file = value(); }
                else if (arg == "--console") { adam_mode = 0; }
                else if (arg == "--adam") { adam_mode = 1; }
                else if (arg == "--exp-ram") {
                        std::string v = value();
                        // Same order as the OSD's Expansion RAM option, so a sim run and a
                        // hardware setting mean the same card. Anything unrecognised is 64K,
                        // which is what the core has always defaulted to.
                        if (v == "256" || v == "256K") exp_ram_mode = 1;
                        else if (v == "512" || v == "512K") exp_ram_mode = 2;
                        else if (v == "1024" || v == "1M") exp_ram_mode = 3;
                        else if (v == "2048" || v == "2M") exp_ram_mode = 4;
                        else if (v == "0" || v == "none") exp_ram_mode = 5;
                        else exp_ram_mode = 0;
                }
                else if (arg == "--headless") { headless = true; }
                else if (arg == "--record") {
                        std::string path = value();
                        record_fp = fopen(path.c_str(), "w");
                        if (!record_fp) { fprintf(stderr, "cannot write %s\n", path.c_str()); exit(1); }
                }
                else if (arg == "--replay") { loadReplay(value()); }
                else if (arg == "--capture-dir") { capture_dir = value(); }
                else if (arg == "--capture-frames") {
                        std::string v = value();
                        size_t dash = v.find('-', 1);
                        capture_from = atoi(v.c_str());
                        capture_to = dash == std::string::npos ? capture_from
                                                               : atoi(v.c_str() + dash + 1);
                }
                else if (arg == "--frames") { run_frames = atoi(value().c_str()); }
                else if (arg == "--every") { shot_every = atoi(value().c_str()); }
                else if (arg == "--outdir") { shot_dir = value(); }
                else if (arg == "--shots") {
                        std::stringstream list(value());
                        std::string frame;
                        while (std::getline(list, frame, ',')) { shot_frames.push_back(atoi(frame.c_str())); }
                }
                else if (arg == "--press") {
                        std::string spec = value();
                        size_t at = spec.find('@');
                        KeyPress p;
                        p.bit = at == std::string::npos ? -1 : keyBit(spec.substr(0, at));
                        if (p.bit < 0) { fprintf(stderr, "Bad --press %s\n", spec.c_str()); exit(1); }
                        p.frame = atoi(spec.c_str() + at + 1);
                        size_t colon = spec.find(':', at);
                        p.length = colon == std::string::npos ? 8 : atoi(spec.c_str() + colon + 1);
                        key_presses.push_back(p);
                }
                else if (arg == "--spin" || arg == "--spin2") {
                        std::string spec = value();
                        size_t at = spec.find('@');
                        if (at == std::string::npos) { fprintf(stderr, "Bad %s %s\n", arg.c_str(), spec.c_str()); exit(1); }
                        SpinSpec s;
                        s.port = (arg == "--spin2") ? 1 : 0;
                        s.steps = atoi(spec.c_str());
                        s.frame = atoi(spec.c_str() + at + 1);
                        size_t colon = spec.find(':', at);
                        s.length = colon == std::string::npos ? 1 : atoi(spec.c_str() + colon + 1);
                        spins.push_back(s);
                        spin_mode_opt = 1;
                }
                else if (arg == "--peek") {
                        std::string spec = value();
                        size_t at = spec.find('@');
                        if (at == std::string::npos) { fprintf(stderr, "Bad --peek %s\n", spec.c_str()); exit(1); }
                        PeekSpec p;
                        p.vram = spec.compare(0, 2, "v:") == 0;
                        if (p.vram) { spec = spec.substr(2); at -= 2; }
                        p.addr = (int)strtol(spec.c_str(), nullptr, 16);
                        size_t colon = spec.find(':');
                        p.len = (colon == std::string::npos || colon > at) ? 1 : atoi(spec.c_str() + colon + 1);
                        p.frame = atoi(spec.c_str() + at + 1);
                        peeks.push_back(p);
                }
                else if (arg == "--disk" || arg == "--tape") {
                        int n = atoi(value().c_str());
                        std::string file = value();
                        if (n < 1 || n > 4) { fprintf(stderr, "%s drive must be 1-4\n", arg.c_str()); exit(1); }
                        (arg == "--disk" ? disk_file : tape_file)[n - 1] = file;
                }
                else if (arg == "--type" || arg == "--key") {
                        std::string spec = value();
                        size_t at = spec.rfind('@');
                        if (at == std::string::npos) { fprintf(stderr, "Bad %s %s\n", arg.c_str(), spec.c_str()); exit(1); }
                        int frame = atoi(spec.c_str() + at + 1);
                        std::string text = spec.substr(0, at);
                        if (arg == "--key") {
                                int code = adamKeyName(text);
                                if (code < 0) { fprintf(stderr, "Unknown key %s\n", text.c_str()); exit(1); }
                                queueAdamKey(code, frame);
                        }
                        else {
                                int slot = 0;
                                for (size_t i = 0; i < text.size(); i++, slot++) {
                                        int c = (unsigned char)text[i];
                                        if (c == '\\' && i + 1 < text.size() && text[i + 1] == 'n') { c = 0x0D; i++; }
                                        if (!queueAdamKey(c, frame + slot * kKeyGap)) { fprintf(stderr, "Cannot type '%c'\n", c); exit(1); }
                                }
                        }
                }
                else if (arg[0] == '+') { continue; }   // Verilator +args
                else { fprintf(stderr, "Unknown option %s\n", arg.c_str()); usage(argv[0]); exit(1); }
        }

        if (headless && run_frames <= 0) { fprintf(stderr, "--headless needs --frames\n"); exit(1); }
        if (!cart_file.empty()) {
                FILE* f = fopen(cart_file.c_str(), "rb");
                if (!f) { fprintf(stderr, "Cannot open cartridge %s\n", cart_file.c_str()); exit(1); }
                fclose(f);
        }
        for (int i = 0; i < 8; i++) {
                const std::string& file = i < 4 ? disk_file[i] : tape_file[i - 4];
                if (file.empty()) { continue; }
                FILE* f = fopen(file.c_str(), "r+b");
                if (!f) { fprintf(stderr, "Cannot open %s for reading and writing\n", file.c_str()); exit(1); }
                fclose(f);
        }
        std::stable_sort(ps2_events.begin(), ps2_events.end(),
                         [](const PS2Event& a, const PS2Event& b) { return a.frame < b.frame; });
}

int runHeadless() {
        video.InitialiseHeadless();
        if (const char* e = getenv("SIM_ADDR_PROFILE")) {
                addr_profile_every = atoi(e);
                addr_hist.assign(65536, 0);
        }
        if (const char* e = getenv("SIM_ADDR_PROFILE_FROM")) { addr_profile_from = atoi(e); }
        if (getenv("SIM_MEGA_TRACE")) { mega_trace = 1; }
        if (getenv("SIM_VDP_TRACE")) { vdp_trace = 1; }
        if (getenv("SIM_SPR5_PROFILE") || mega_trace || vdp_trace) {
                spr5_profile = 1;
                spr5_detect_hist.assign(32, 0);
                spr5_line_min.assign(32, 9999);
                spr5_line_max.assign(32, -9999);
                mega_hist.assign(64, 0);
                spr5_latched_hist.assign(32, 0);
        }
        VERTOPINTERN->joystick_0 = scriptedJoystick(0);

        auto start = std::chrono::steady_clock::now();
        while (!exit_requested) { stepSim(); }
        double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

        if (addr_profile_every > 0) {
                std::vector<int> order(65536);
                for (int i = 0; i < 65536; i++) { order[i] = i; }
                std::partial_sort(order.begin(), order.begin() + 24, order.end(),
                                  [](int a, int b) { return addr_hist[a] > addr_hist[b]; });
                unsigned long total = 0;
                for (unsigned long n : addr_hist) { total += n; }
                printf("address profile (%lu samples):\n", total);
                for (int i = 0; i < 24 && addr_hist[order[i]]; i++) {
                        printf("  %04X %6.2f%%\n", order[i], 100.0 * addr_hist[order[i]] / total);
                }
        }

        if (spr5_profile) {
                printf("fifth sprite: %lu detections in %lu steps, flag set for %.2f%% of them\n",
                       spr5_events, spr5_steps, 100.0 * spr5_flag_steps / (spr5_steps ? spr5_steps : 1));
                printf("  numbers reported by the sprite engine, with the scanlines they came on:");
                for (int i = 0; i < 32; i++) {
                        if (spr5_detect_hist[i]) {
                                printf(" %d:%lu@%d", i, spr5_detect_hist[i], spr5_line_min[i]);
                                if (spr5_line_max[i] != spr5_line_min[i]) { printf("..%d", spr5_line_max[i]); }
                        }
                }
                printf("\n  numbers visible in the status register:");
                for (int i = 0; i < 32; i++) { if (spr5_latched_hist[i]) { printf(" %d:%lu", i, spr5_latched_hist[i]); } }
                printf("\nvram writes: %lu scheduled, %lu of them while the previous one was still pending\n",
                       vram_write_reqs, vram_write_collisions);
                printf("  wait for an access slot, in 372ns memory cycles: <=1:%lu 2-3:%lu 4-7:%lu 8-15:%lu 16+:%lu max:%lu\n",
                       vram_wait_hist[0], vram_wait_hist[1], vram_wait_hist[2],
                       vram_wait_hist[3], vram_wait_hist[4], vram_wait_max);
                printf("vdp interrupt: %lu assertions, last at frame %d of %d\n",
                       vdp_int_falls, vdp_int_last_frame, last_frame);
                printf("  status reads that should have cleared the flag: %lu, flag actually cleared %lu times"
                       " -> %ld clears lost to the set/clear race\n",
                       vdp_clearing_reads, vdp_int_rises,
                       (long)vdp_clearing_reads - (long)vdp_int_rises);
                printf("megacart: %lu bank switches, last at frame %d; pages used:",
                       mega_switches, mega_last_frame);
                for (int i = 0; i < 64; i++) { if (mega_hist[i]) { printf(" %d", i); } }
                printf("\n");
        }

        printf("frames=%d main_time=%llu seconds=%.1f fps=%.2f\n", last_frame, (unsigned long long)main_time, seconds, last_frame / seconds);
        top->final();
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
        parseArgs(argc, argv);
        Verilated::traceEverOn(true);
ROMPage[0] = (Byte *)&VERTOPINTERN->emu__DOT__ram__DOT__ram;
ROMPage[1] = ROMPage[0]+0x2000;
ROMPage[2] = ROMPage[1]+0x2000;
ROMPage[3] = ROMPage[2]+0x2000;

ROMPage[4] = (Byte *)&VERTOPINTERN->emu__DOT__upper_ram__DOT__ram;
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
        bus.ioctl_addr = &VERTOPINTERN->ioctl_addr;
        bus.ioctl_index = &VERTOPINTERN->ioctl_index;
        bus.ioctl_wait = &VERTOPINTERN->ioctl_wait;
        bus.ioctl_download = &VERTOPINTERN->ioctl_download;
        //bus.ioctl_upload = &VERTOPINTERN->ioctl_upload;
        bus.ioctl_wr = &VERTOPINTERN->ioctl_wr;
        bus.ioctl_dout = &VERTOPINTERN->ioctl_dout;
        //bus.ioctl_din = &VERTOPINTERN->ioctl_din;
        input.ps2_key = &VERTOPINTERN->ps2_key;

        // hookup blk device
        //blockdevice.MountDisk("adam.dsk",0);
        blockdevice.sd_lba[0] = &VERTOPINTERN->sd_lba[0];
        blockdevice.sd_lba[1] = &VERTOPINTERN->sd_lba[1];
        blockdevice.sd_lba[2] = &VERTOPINTERN->sd_lba[2];
        blockdevice.sd_lba[3] = &VERTOPINTERN->sd_lba[3];
        blockdevice.sd_lba[4] = &VERTOPINTERN->sd_lba[4];
        blockdevice.sd_lba[5] = &VERTOPINTERN->sd_lba[5];
        blockdevice.sd_lba[6] = &VERTOPINTERN->sd_lba[6];
        blockdevice.sd_lba[7] = &VERTOPINTERN->sd_lba[7];
        blockdevice.sd_rd = &VERTOPINTERN->sd_rd;
        blockdevice.sd_wr = &VERTOPINTERN->sd_wr;
        blockdevice.sd_ack = &VERTOPINTERN->sd_ack;
        blockdevice.sd_buff_addr= &VERTOPINTERN->sd_buff_addr;
        blockdevice.sd_buff_dout= &VERTOPINTERN->sd_buff_dout;
        // every drive writes through its own buffer: disks 0-3, tapes 4-7 (only 0 and 1 used to be
        // wired, so the first write to drive 2-7 dereferenced NULL)
        for (int i = 0; i < 8; i++) blockdevice.sd_buff_din[i] = &VERTOPINTERN->sd_buff_din[i];
        blockdevice.sd_buff_wr= &VERTOPINTERN->sd_buff_wr;
        blockdevice.img_mounted= &VERTOPINTERN->img_mounted;
        blockdevice.img_readonly= &VERTOPINTERN->img_readonly;
        blockdevice.img_size= &VERTOPINTERN->img_size;

        // Media from the command line; otherwise adam.dsk and adam.ddp from the current directory
        bool media = false;
        for (int i = 0; i < 4; i++) {
                if (!disk_file[i].empty()) { blockdevice.MountDisk(disk_file[i], i); media = true; }
                if (!tape_file[i].empty()) { blockdevice.MountDisk(tape_file[i], 4 + i); media = true; }
        }
        if (!media) {
                blockdevice.MountDisk("adam.dsk",0);
                blockdevice.MountDisk("adam.ddp",4);
        }

        // Cartridge from the command line; index 1 is the core's "Load CART"
        if (!cart_file.empty()) { bus.QueueDownload(cart_file, 1, 1); }


#ifndef DISABLE_AUDIO
        audio.Initialise();
#endif

        if (headless) { return runHeadless(); }

        // Set up input module
        input.Initialise();
        // Every index has to be set, because SetMapping only writes the ones it is given and the
        // joystick loop reads all of them.
        for (int i = 0; i < input.inputCount; i++) { input.SetMapping(i, 0); }
#ifdef WIN32
        input.SetMapping(input_up, DIK_UP);
        input.SetMapping(input_right, DIK_RIGHT);
        input.SetMapping(input_down, DIK_DOWN);
        input.SetMapping(input_left, DIK_LEFT);
        input.SetMapping(input_fire1, DIK_A);
        input.SetMapping(input_fire2, DIK_B);
        input.SetMapping(input_menu, DIK_M);
#else
        // Arrows and A/B are what this simulator has always used. The number row now sends the
        // ColecoVision keypad digit it is printed with, which it did not before: "1" used to send
        // keypad 3 and keypad 1 was on the E key, so a game asking for "press 1" needed E.
        input.SetMapping(input_up, SDL_SCANCODE_UP);
        input.SetMapping(input_right, SDL_SCANCODE_RIGHT);
        input.SetMapping(input_down, SDL_SCANCODE_DOWN);
        input.SetMapping(input_left, SDL_SCANCODE_LEFT);
        input.SetMapping(input_fire1, SDL_SCANCODE_A);
        input.SetMapping(input_fire2, SDL_SCANCODE_B);
        input.SetMapping(input_star, SDL_SCANCODE_COMMA);
        input.SetMapping(input_pound, SDL_SCANCODE_PERIOD);
        input.SetMapping(input_kp0, SDL_SCANCODE_0);
        for (int d = 1; d <= 9; d++) {
                input.SetMapping(input_kp0 + d, SDL_SCANCODE_1 + (d - 1));
        }
        input.SetMapping(input_purple, SDL_SCANCODE_P);
        input.SetMapping(input_blue, SDL_SCANCODE_U);
        input.SetMapping(input_menu, SDL_SCANCODE_M);
        // Square brackets and backslash rather than function keys, which a Mac laptop puts
        // behind fn.
        input.SetMapping(input_cap_start, SDL_SCANCODE_LEFTBRACKET);
        input.SetMapping(input_cap_stop,  SDL_SCANCODE_RIGHTBRACKET);
        input.SetMapping(input_cap_one,   SDL_SCANCODE_BACKSLASH);
        input.SetMapping(input_p2_up,     SDL_SCANCODE_I);
        input.SetMapping(input_p2_left,   SDL_SCANCODE_J);
        input.SetMapping(input_p2_down,   SDL_SCANCODE_K);
        input.SetMapping(input_p2_right,  SDL_SCANCODE_L);
        input.SetMapping(input_p2_fire1,  SDL_SCANCODE_F);
        input.SetMapping(input_p2_fire2,  SDL_SCANCODE_G);
#endif
        // Setup video output
        if (video.Initialise(windowTitle) == 1) { return 1; }

        //bus.QueueDownload("floppy.nib",1,0);
        //blockdevice.MountDisk("floppy.nib",0);
        //blockdevice.MountDisk("hd.hdv",1);

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
    ImGuiFileDialog::Instance()->OpenDialog("ChooseFileDlgKey", "Choose File", ".col,.rom,.bin", ".");

                if (ImGui::Button("PRINT MARKER")) { fprintf(stderr,"7F MARKER\n"); fprintf(stdout,"7F MARKER\n");  } ImGui::SameLine();
                //if (ImGui::Button("Soft Reset")) { fprintf(stderr,"soft reset\n"); soft_reset=1; } ImGui::SameLine();

                ImGui::End();

                // Debug log window
                console.Draw(windowTitle_DebugLog, &showDebugLog, ImVec2(500, 700));
                ImGui::SetWindowPos(windowTitle_DebugLog, ImVec2(0, 160), ImGuiCond_Once);

                // Memory debug
                ImGui::Begin("ram Editor");
                mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__ram__DOT__ram , 32768, 0);
                ImGui::End();
                ImGui::Begin("upper ram Editor");
                mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__upper_ram__DOT__ram, 32768, 0);
                ImGui::End();
                ImGui::Begin("expansion RAM Editor");   // {bank(2), upper window(1), address(15)}
                mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__expansion_ram__DOT__mem, 262144, 0);
                ImGui::End();
                //ImGui::Begin("CHROM Editor");
                //mem_edit.DrawContents(VERTOPINTERN->emu__DOT__system__DOT__chrom__DOT__mem, 2048, 0);
                //ImGui::End();
                //ImGui::Begin("WKRAM Editor");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__wkram__DOT__mem, 16384, 0);
                //ImGui::End();
                //ImGui::Begin("CHRAM Editor");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__chram__DOT__mem, 2048, 0);
                //ImGui::End();
                //ImGui::Begin("FGCOLRAM Editor");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__fgcolram__DOT__mem, 2048, 0);
                //ImGui::End();
                //ImGui::Begin("BGCOLRAM Editor");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__bgcolram__DOT__mem, 2048, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite RAM");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__spriteram__DOT__mem, 96, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite Linebuffer RAM");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__spritelbram__DOT__mem, 1024, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite Collision Buffer RAM A");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__comet__DOT__spritecollisionbufferram_a__DOT__mem, 512, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite Collision Buffer RAM B");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__comet__DOT__spritecollisionbufferram_b__DOT__mem, 512, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite Collision RAM ");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__spritecollisionram__DOT__mem, 32, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite Debug RAM");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__spritedebugram__DOT__mem, 128000, 0);
                //ImGui::End();
                //ImGui::Begin("Palette ROM");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__palrom__DOT__mem, 64, 0);
                //ImGui::End();
                //ImGui::Begin("Sprite ROM");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__spriterom__DOT__mem, 2048, 0);
                //ImGui::End();
                //ImGui::Begin("Tilemap ROM");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__tilemaprom__DOT__mem, 8192, 0);
                //ImGui::End();
                //ImGui::Begin("Tilemap RAM");
                //	mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__tilemapram__DOT__mem, 768, 0);
                //ImGui::End();
                //ImGui::Begin("Sound ROM");
                //mem_edit.DrawContents(&VERTOPINTERN->emu__DOT__system__DOT__soundrom__DOT__mem, 64000, 0);
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


                //float vol_l = ((signed short)(VERTOPINTERN->AUDIO_L) / 256.0f) / 256.0f;
                //float vol_r = ((signed short)(VERTOPINTERN->AUDIO_R) / 256.0f) / 256.0f;
                //ImGui::ProgressBar(vol_l + 0.5f, ImVec2(200, 16), 0); ImGui::SameLine();
                //ImGui::ProgressBar(vol_r + 0.5f, ImVec2(200, 16), 0);

                int ticksPerSec = (24000000 / 60);
                if (run_enable) {
                        audio.CollectDebug((signed short)VERTOPINTERN->AUDIO_L, (signed short)VERTOPINTERN->AUDIO_R);
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

                VERTOPINTERN->menu = input.inputs[input_menu];

                VERTOPINTERN->joystick_0 = 0;
                // Only the first input_joy_bits indices are joystick bits; input_menu is not.
                for (int i = 0; i < input_joy_bits; i++)
                {
                        if (input.inputs[i]) { VERTOPINTERN->joystick_0 |= (1 << i); }
                }
                // Player 2's six keys sit at the end of the input list and map onto the bottom
                // six bits of the second controller: right, left, down, up, fire 1, fire 2.
                VERTOPINTERN->joystick_1 = 0;
                for (int i = 0; i < 6; i++)
                {
                        if (input.inputs[input_p2_right + i]) { VERTOPINTERN->joystick_1 |= (1 << i); }
                }
                uint32_t rp1 = 0, rp2 = 0;
                replayJoystick(video.count_frame, &rp1, &rp2);
                VERTOPINTERN->joystick_0 |= scriptedJoystick(video.count_frame) | rp1;
                VERTOPINTERN->joystick_1 |= rp2;
                recordJoystick(video.count_frame, VERTOPINTERN->joystick_0, VERTOPINTERN->joystick_1);

                /*VERTOPINTERN->joystick_analog_0 += 1;
                VERTOPINTERN->joystick_analog_0 -= 256;*/
                //VERTOPINTERN->paddle_0 += 1;
                //if (input.inputs[0] || input.inputs[1]) {
                //	spinner_toggle = !spinner_toggle;
                //	VERTOPINTERN->spinner_0 = (input.inputs[0]) ? 16 : -16;
                //	for (char b = 8; b < 16; b++) {
                //		VERTOPINTERN->spinner_0 &= ~(1UL << b);
                //	}
                //	if (spinner_toggle) { VERTOPINTERN->spinner_0 |= 1UL << 8; }
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

                VERTOPINTERN->ps2_mouse = mouse_temp;
                VERTOPINTERN->ps2_mouse_ext = mouse_x + (mouse_buttons << 8);

                // Run simulation
                if (run_enable) {
                        for (int step = 0; step < batchSize; step++) { stepSim(); }
                }
                else {
                        if (single_step) { stepSim(); }
                        if (multi_step) {
                                for (int step = 0; step < multi_step_amount; step++) { stepSim(); }
                        }
                }
                if (exit_requested) { break; }
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
