// Testbench for rtl/sdram.sv, run against the behavioural chip in sdram_model.sv.
//
// The core's own simulator cannot check this controller: it clocks clk_sys at the
// 10.7 MHz rate with ce_10m7 tied high, so a seven clock SDRAM slot there would be
// two and a half Z80 clocks long instead of the half a clock it is on hardware,
// and every access would look like a stall. Here the clock is the real 42.666 MHz
// one and the controller can be checked on its own terms.
//
// What is checked:
//   1  data written on either channel reads back, across bank, row and column bits
//   2  a read of the byte beside one already fetched is answered from the cached
//      word without the CPU being made to wait - the property the cartridge path
//      depends on, and the reason moving the expander in here is free
//   3  the two channels keep separate caches, so interleaving them does not thrash
//   4  a read queued behind a write on the same channel is answered from memory,
//      not from the write's address
//   5  refresh keeps happening at roughly the rate the part needs

#include <verilated.h>
#include "Vtb_top.h"
#include <cstdio>
#include <cstdlib>
#include <vector>

static Vtb_top* top;
static vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

static int failures = 0;
static long cycles = 0;

static void tick(int n = 1) {
        for (int i = 0; i < n; i++) {
                top->clk = 0; top->eval();
                top->clk = 1; top->eval();
                cycles++;
                main_time++;
        }
}

static void check(bool ok, const char* what, long got = 0, long want = 0) {
        if (ok) return;
        failures++;
        printf("  FAIL %s (got %ld, wanted %ld)\n", what, got, want);
}

// One access, driven the way the core drives it: raise the strobe, leave it up
// until the channel says it is ready, then drop it. Returns the cycles spent
// waiting, which is 0 when the answer came out of the cached word.
static int rd(int ch, unsigned addr, unsigned char* out) {
        if (ch == 0) { top->ch0_addr = addr; top->ch0_rd = 1; }
        else         { top->ch1_addr = addr; top->ch1_rd = 1; }
        tick(2);                       // the strobe is seen and the request registers
        int waited = 0;
        while (!(ch == 0 ? top->ch0_ready : top->ch1_ready)) {
                tick();
                if (++waited > 200) { check(false, "read never completed"); break; }
        }
        *out = (ch == 0) ? top->ch0_dout : top->ch1_dout;
        if (ch == 0) top->ch0_rd = 0; else top->ch1_rd = 0;
        tick();
        return waited;
}

static void wr(int ch, unsigned addr, unsigned char v) {
        if (ch == 0) { top->ch0_addr = addr; top->ch0_din = v; top->ch0_wr = 1; }
        else         { top->ch1_addr = addr; top->ch1_din = v; top->ch1_wr = 1; }
        tick(2);
        int waited = 0;
        while (!(ch == 0 ? top->ch0_ready : top->ch1_ready)) {
                tick();
                if (++waited > 200) { check(false, "write never completed"); break; }
        }
        if (ch == 0) top->ch0_wr = 0; else top->ch1_wr = 0;
        tick();
}

int main(int argc, char** argv) {
        Verilated::commandArgs(argc, argv);
        top = new Vtb_top;

        top->clk = 0; top->init = 1;
        top->ch0_addr = 0; top->ch0_rd = 0; top->ch0_wr = 0; top->ch0_din = 0;
        top->ch1_addr = 0; top->ch1_rd = 0; top->ch1_wr = 0; top->ch1_din = 0;
        tick(10);
        top->init = 0;

        // The controller waits out the part's power up time before it will answer.
        int boot = 0;
        while (!top->ch0_ready) {
                tick();
                if (++boot > 100000) { printf("  FAIL controller never came ready\n"); failures++; break; }
        }
        printf("boot: ready after %d cycles (%.0f us at 42.666 MHz)\n", boot, boot / 42.666);

        // --- 1: data survives a round trip on both channels ----------------------
        // Addresses are spread so that the bank, row and column fields all change:
        // the controller puts a[13:1] in the row, a[22:14] in the column and
        // a[24:23] in the bank, so a mapping mistake shows up as a collision.
        printf("test 1: write and read back, both channels\n");
        struct Cell { int ch; unsigned addr; unsigned char val; };
        std::vector<Cell> cells;
        unsigned char v = 1;
        for (unsigned i = 0; i < 64; i++) {
                unsigned a0 = (i * 0x4321u) & 0x0FFFFFu;              // cartridge region
                unsigned a1 = 0x200000u + ((i * 0x7919u) & 0x1FFFFFu);// expander region
                cells.push_back({0, a0, v++});
                cells.push_back({1, a1, v++});
        }
        for (auto& c : cells) wr(c.ch, c.addr, c.val);
        for (auto& c : cells) {
                unsigned char got;
                rd(c.ch, c.addr, &got);
                if (got != c.val) {
                        failures++;
                        printf("  FAIL ch%d %06X: got %02X wanted %02X\n", c.ch, c.addr, got, c.val);
                        if (failures > 8) { printf("  (stopping after 8)\n"); break; }
                }
        }

        // --- 2: the cached word answers the neighbouring byte for free -----------
        printf("test 2: sequential reads hit the cached word\n");
        for (unsigned i = 0; i < 64; i++) wr(0, 0x001000u + i, (unsigned char)(0xA0 + i));
        int hits = 0, misses = 0;
        for (unsigned i = 0; i < 64; i++) {
                unsigned char got;
                int waited = rd(0, 0x001000u + i, &got);
                if (got != (unsigned char)(0xA0 + i)) {
                        failures++;
                        printf("  FAIL sequential %04X: got %02X wanted %02X\n", 0x1000 + i, got, 0xA0 + i);
                        break;
                }
                if (waited == 0) hits++; else misses++;
        }
        printf("  %d hits, %d misses out of 64\n", hits, misses);
        // Every odd byte shares a 16 bit word with the even one before it.
        check(hits >= 31, "half the sequential reads should cost nothing", hits, 32);

        // --- 3: the channels do not evict each other -----------------------------
        printf("test 3: interleaved channels keep separate caches\n");
        for (unsigned i = 0; i < 32; i++) {
                wr(0, 0x002000u + i, (unsigned char)(0x10 + i));
                wr(1, 0x202000u + i, (unsigned char)(0x50 + i));
        }
        int both_hits = 0;
        for (unsigned i = 0; i < 32; i++) {
                unsigned char g0, g1;
                int w0 = rd(0, 0x002000u + i, &g0);
                int w1 = rd(1, 0x202000u + i, &g1);
                if (g0 != (unsigned char)(0x10 + i) || g1 != (unsigned char)(0x50 + i)) {
                        failures++;
                        printf("  FAIL interleaved %u: ch0 %02X ch1 %02X\n", i, g0, g1);
                        break;
                }
                if (w0 == 0) both_hits++;
                if (w1 == 0) both_hits++;
        }
        printf("  %d free reads out of 64\n", both_hits);
        // With one cache between them this would be 0: each access would evict the
        // other's word and every read would go to the chip.
        check(both_hits >= 30, "interleaving must not thrash the caches", both_hits, 32);

        // --- 4: a read queued behind a write on the same channel -----------------
        // The Z80 writes a marker and reads it straight back; if a refresh delays
        // the write the two are outstanding together. Drive them back to back
        // without waiting in between, which is what makes that happen here.
        printf("test 4: read queued behind a write\n");
        for (unsigned i = 0; i < 32; i++) {
                unsigned addr = 0x210000u + i * 2u;
                unsigned char want = (unsigned char)(0xC0 + i);
                top->ch1_addr = addr; top->ch1_din = want; top->ch1_wr = 1;
                tick(2);
                top->ch1_wr = 0;
                top->ch1_addr = addr; top->ch1_rd = 1;   // queued while the write is in flight
                tick(2);
                int waited = 0;
                while (!top->ch1_ready) { tick(); if (++waited > 200) break; }
                unsigned char got = top->ch1_dout;
                top->ch1_rd = 0;
                tick();
                if (got != want) {
                        failures++;
                        printf("  FAIL write-then-read %06X: got %02X wanted %02X\n", addr, got, want);
                        break;
                }
        }

        // --- 5: refresh -----------------------------------------------------------
        printf("test 5: refresh rate\n");
        unsigned before = top->refresh_count;
        long at = cycles;
        tick(40000);                   // 937 us
        long elapsed = cycles - at;
        unsigned did = top->refresh_count - before;
        double want = (double)elapsed / 333.0;
        printf("  %u refreshes in %ld cycles, wanted about %.0f\n", did, elapsed, want);
        // 8192 rows every 64 ms is one every 333 clocks; falling short loses data.
        check(did >= want * 0.95, "refresh must keep up", did, (long)want);

        printf(failures ? "\nFAILED: %d checks\n" : "\nall checks passed\n", failures);
        delete top;
        return failures ? 1 : 0;
}
