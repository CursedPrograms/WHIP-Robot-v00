// serial_emu - WHIP's servo serial, emulated on a PC: the exact lines his ESP32 sends the RTrobot
// controller ("#<ch>P<us>...T<ms>") for every movement, from his own IK and gait code (gaits.h,
// ik_gait.h), timed and clamped as esp32_gaits.ino's gaitCycle() / gaitSettle() / ikSend() do.
// serial_to_xml.py --emulate builds and runs it, and turns the lines into RTrobot XML macros.
//
//   serial_emu [cal.txt]     cal.txt: "height speed" on the first line, then 18 lines
//                            "ch rev trim" (LF LM LB RF RM RB; hip knee foot). None: his defaults.
// Output: "= <macro name>" before each movement, then its serial lines.
#include "gaits.h"
using namespace ik;

const unsigned long CONTROLLER_BAUD = 115200;   // as esp32_gaits.ino
const uint16_t LINE_MS = (uint16_t)((145UL * 10UL * 1000UL) / CONTROLLER_BAUD) + 5;
const int US_MIN = 500, US_MAX = 2500;           // clampCommand()

uint16_t airTicks() { return 2 * (LINE_MS > stepMs() ? walkSteps() : STEPS); }

// ikSend(): the IK line, every pulse kept in 500-2500 us, then T<ms>
void send(const Frame& f, uint16_t ms) {
  float a[JOINTS];
  for (uint8_t leg = 0; leg < LEGS; leg++) {
    if (!solve(leg, f.leg[leg], a)) { printf("! foot out of reach, frame skipped\n"); return; }
    for (uint8_t j = 0; j < JOINTS; j++)
      printf("#%uP%d", cal[leg][j].ch, constrain(toUs(leg, j, a[j]), US_MIN, US_MAX));
  }
  printf("T%u\n", ms);
}

void zero(uint16_t ms) {
  for (uint8_t leg = 0; leg < LEGS; leg++)
    for (uint8_t j = 0; j < JOINTS; j++) printf("#%uP%d", cal[leg][j].ch, constrain(toUs(leg, j, 0), US_MIN, US_MAX));
  printf("T%u\n", ms);
}

// From standing: `cycles` of gaitCycle(), then gaitSettle() (he steps his feet home)
void walk(const char* name, uint8_t gait, float fwd, float left, float turn, int cycles) {
  printf("= %s %s\n", GAIT_NAMES[gait], name);
  Gait G;
  G.start(home(), gait, airTicks());
  const uint16_t ms = max(stepMs(), LINE_MS);
  for (int c = 0; c < cycles; c++)
    for (uint16_t i = 0; i < G.cycleTicks; i++) send(G.tick(fwd, left, turn, false), ms);
  for (uint16_t i = 0; i < 4 * G.cycleTicks && !G.settled(); i++) send(G.tick(0, 0, 0, true), ms);
}

int main(int argc, char** argv) {
  loadDefaults();
  if (argc > 1) {
    FILE* f = fopen(argv[1], "r");
    if (!f) { fprintf(stderr, "can't open %s\n", argv[1]); return 1; }
    int h, s;
    if (fscanf(f, "%d %d", &h, &s) == 2) { height = constrain(h, 0, 9); speed = constrain(s, 0, 9); }
    for (uint8_t leg = 0; leg < LEGS; leg++)
      for (uint8_t j = 0; j < JOINTS; j++) {
        int ch, rev, trim;
        if (fscanf(f, "%d %d %d", &ch, &rev, &trim) == 3) cal[leg][j] = { (uint8_t)ch, rev != 0, (int16_t)trim };
      }
    fclose(f);
  }
  printf("= stand\n");
  send(home(), 300);            // ikSnapHome()
  printf("= zero\n");
  zero(600);                    // /calib ZERO
  struct { const char* name; float f, l, t; } moves[] = {
    { "forward", 1, 0, 0 }, { "backward", -1, 0, 0 }, { "turn left", 0, 0, 1 },
    { "turn right", 0, 0, -1 }, { "crab left", 0, 1, 0 }, { "crab right", 0, -1, 0 },
  };
  for (uint8_t g = 0; g < GAITS; g++)
    for (auto& m : moves) walk(m.name, g, m.f, m.l, m.t, 2);
  return 0;
}
