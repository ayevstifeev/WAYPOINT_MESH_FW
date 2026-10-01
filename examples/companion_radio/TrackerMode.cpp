#include "TrackerMode.h"

#ifdef WITH_TRACKER_MODE

#include <helpers/slim/SlimCodecs.h>

namespace tracker {

// Unsigned subtraction, so millis() wrapping after 49 days is not a special case.
static uint32_t since(uint32_t now, uint32_t then) { return now - then; }

bool validInterval(uint32_t s) {
  return s == 0 || (s >= MIN_INTERVAL_S && s <= MAX_INTERVAL_S);
}

bool parseCommand(const char* text, uint32_t& seconds) {
  if (text == nullptr || text[0] != '!' || text[1] != 'T') return false;
  uint32_t n = 0;
  int digits = 0;
  for (const char* p = text + 2; *p; p++) {
    // Five digits covers the largest interval with room to refuse one that is
    // too big, rather than letting it wrap into one that looks valid.
    if (*p < '0' || *p > '9' || ++digits > 5) return false;
    n = n * 10 + (uint32_t)(*p - '0');
  }
  if (digits == 0) return false;
  seconds = n;
  return true;
}

bool parseFindCommand(const char* text, uint32_t& minutes) {
  if (text == nullptr || text[0] != '!' || text[1] != 'F') return false;
  uint32_t n = 0;
  int digits = 0;
  for (const char* p = text + 2; *p; p++) {
    if (*p < '0' || *p > '9' || ++digits > 4) return false;
    n = n * 10 + (uint32_t)(*p - '0');
  }
  if (digits == 0) return false;
  minutes = n;
  return true;
}

// ---- the family channel -----------------------------------------------------

static char upper(char c) {
  return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
}

bool isParentName(const char* name) {
  if (name == nullptr) return false;
  for (int i = 0; i < 4; i++) {
    if (upper(name[i]) != "MCMD"[i]) return false;
  }
  return name[4] == 0;
}

bool targetMatches(const char* target, const char* self_name) {
  if (target == nullptr || self_name == nullptr) return false;
  if (target[0] == '*' && target[1] == 0) return true;
  //: Bytes, not characters: a name may begin with an emoji - the face the
  //: finder shows - and upper() leaves every byte above ASCII alone, so the
  //: comparison is exact there and case-insensitive on the letters.
  const char *a = target, *b = self_name;
  for (; *a && *b; a++, b++) {
    if (upper(*a) != upper(*b)) return false;
  }
  return *a == 0 && *b == 0;
}

bool parseChannelCommand(const char* text, ChannelCmd& out) {
  if (text == nullptr) return false;
  //: MeshCore writes "NAME: text" into every group message, because a group
  //: packet is encrypted to the channel and carries no sender of its own.
  const char* colon = nullptr;
  for (const char* p = text; *p; p++) {
    if (p[0] == ':' && p[1] == ' ') { colon = p; break; }
  }
  if (colon == nullptr) return false;
  const size_t name_len = (size_t)(colon - text);
  if (name_len == 0 || name_len >= sizeof(out.sender)) return false;
  for (size_t i = 0; i < name_len; i++) out.sender[i] = text[i];
  out.sender[name_len] = 0;

  const char* body = colon + 2;
  while (*body == ' ') body++;
  //: "!T300 BELLA" sets the usual interval; "!F30 BELLA" is find mode, and
  //: "!F0 BELLA" ends it. The target is the rest of the line, so a name with
  //: a space in it still arrives whole.
  const char* space = body;
  while (*space && *space != ' ') space++;
  if (*space != ' ') return false;
  char cmd[12];
  const size_t cmd_len = (size_t)(space - body);
  if (cmd_len == 0 || cmd_len >= sizeof(cmd)) return false;
  for (size_t i = 0; i < cmd_len; i++) cmd[i] = body[i];
  cmd[cmd_len] = 0;

  uint32_t value = 0;
  if (parseCommand(cmd, value))            out.find = false;
  else if (parseFindCommand(cmd, value))   out.find = true;
  else return false;
  out.value = value;

  const char* target = space + 1;
  while (*target == ' ') target++;
  size_t n = 0;
  while (target[n] && n < sizeof(out.target) - 1) { out.target[n] = target[n]; n++; }
  //: Trailing blanks rather than a name: a target nobody can match is not a
  //: command to act on.
  while (n > 0 && out.target[n - 1] == ' ') n--;
  out.target[n] = 0;
  return n > 0;
}

// ---- the schedule -----------------------------------------------------------

Schedule::Schedule(uint32_t fix_timeout_ms, uint32_t listen_ms)
  : _phase(OFF), _interval_s(0), _cycle_at(0), _phase_at(0),
    _fix_timeout_ms(fix_timeout_ms), _listen_ms(listen_ms) { }

bool Schedule::start(uint32_t interval_s, uint32_t now) {
  if (interval_s < MIN_INTERVAL_S || interval_s > MAX_INTERVAL_S) return false;
  _interval_s = interval_s;
  _cycle_at = now;
  enter(FIX, now);
  return true;
}

void Schedule::setClock(uint32_t epoch_s, uint32_t now) {
  _epoch_s = epoch_s;
  _clock_at = now;
  _clock_set = epoch_s != 0;
}

uint32_t Schedule::epoch(uint32_t now) const {
  if (!_clock_set) return 0;
  return _epoch_s + since(now, _clock_at) / 1000UL;
}

void Schedule::setSlot(uint8_t unit, uint16_t slot_s, uint16_t lead_s) {
  _slot_unit = (slot_s == 0) ? 0 : unit;
  _slot_s = slot_s;
  _lead_s = lead_s;
}

//: Straight to slimround, not a second copy of the same expression. This
//: file had its own and did not follow when the slots were renumbered,
//: which is exactly the drift that puts two units in one moment with
//: every symptom looking like poor range.
uint32_t Schedule::nextSlot(uint32_t now) const {
  if (!slotted() || _interval_s == 0) return 0;
  return slimround::nextSlot(epoch(now), _interval_s, _slot_unit,
                             (uint8_t)_slot_s);
}

void Schedule::setSamples(uint8_t count, uint16_t cadence_s) {
  _sample_n = (count < 1) ? 1 : count;
  _sample_s = (_sample_n > 1) ? cadence_s : 0;
}

//: The sample instants are the slot and the `n-1` cadence steps before
//: it, so the newest place in a report is always the moment it went out.
uint32_t Schedule::nextSampleAt(uint32_t now) const {
  const uint32_t slot = nextSlot(now);
  if (slot == 0 || _sample_s == 0 || _sample_n <= 1) return slot;
  const uint32_t e = epoch(now);
  const uint32_t first = slot - (uint32_t)(_sample_n - 1) * _sample_s;
  if (e < first) return first;
  if (e >= slot) return slot;
  return first + (((e - first) / _sample_s) + 1) * _sample_s;
}

bool Schedule::step(uint32_t now, bool fix, bool quiet) {
  return tick(now, fix, quiet) == REPORT;
}

Schedule::Action Schedule::tick(uint32_t now, bool fix, bool quiet) {
  switch (_phase) {
    case FIX: {
      if (slotted()) {
        // The GPS was started for one particular moment, and that moment
        // is what this phase is waiting for. Asking "which slot does now
        // fall in" instead would read every instant after this round's
        // slot as one that had been missed - including the whole of the
        // round spent collecting samples for the NEXT one.
        const uint32_t e = epoch(now);
        if (_sample_at == 0) { enter(SLEEP, now); return NOTHING; }
        if (e < _sample_at) return NOTHING;   // early: keep looking
        const uint32_t at = slimround::slotStart(_sample_at, _interval_s,
                                                 _slot_unit,
                                                 (uint8_t)_slot_s);
        const bool is_slot = (_sample_at == at);
        _sample_at = 0;
        enter(is_slot ? LISTEN : SLEEP, now);
        if (!is_slot) {
          // A sample between reports: keep the fix, go back to sleep.
          // Nothing goes on the air, which is the whole saving.
          return fix ? SAMPLE : NOTHING;
        }
        if (e < at + _slot_s) {
          // With a fix if the sky allowed one, and without if it did
          // not: a unit heard with no fix is a radio working under a
          // canopy that is not, and silence is a team nobody can hear.
          return REPORT;
        }
        // Past the window - the GPS held the loop, or the clock moved.
        // Wait for the next one rather than transmitting over whoever
        // owns this moment.
        enter(SLEEP, now);
        return NOTHING;
      }
      if (fix) {
        enter(LISTEN, now);
        return REPORT;
      }
      // Never into the next report's time: with a 60 s interval a 90 s
      // timeout would mean the unit never slept at all.
      uint32_t limit = _fix_timeout_ms;
      const uint32_t budget = _interval_s * 1000UL - _listen_ms;
      if (limit > budget) limit = budget;
      if (since(now, _phase_at) >= limit) {
        // Out of time and still no fix. The report goes anyway: a unit
        // heard with no fix is a radio that works under a canopy that
        // does not, and an operator needs to be able to tell that from a
        // team that has gone quiet.
        enter(LISTEN, now);
        return REPORT;
      }
      return NOTHING;
    }
    case LISTEN:
      // Counted from when the last transmission left, not from the send: three
      // copies of a report and an ack can take seconds at SF12, and the window
      // is for hearing the base's answer to them.
      if (!quiet) _phase_at = now;
      else if (since(now, _phase_at) >= _listen_ms) enter(SLEEP, now);
      return NOTHING;
    case SLEEP:
      if (slotted()) {
        // Start the GPS `lead` seconds before the moment a fix is wanted,
        // so it is ready when that moment comes rather than being chased
        // inside it. With a trail that is every sample, not just the slot.
        const uint32_t e = epoch(now);
        const uint32_t next = nextSampleAt(now);
        if (next != 0 && next - e <= _lead_s) {
          _cycle_at = now;
          _sample_at = next;
          enter(FIX, now);
        }
        return NOTHING;
      }
      if (since(now, _cycle_at) >= _interval_s * 1000UL) {
        _cycle_at = now;
        enter(FIX, now);
      }
      return NOTHING;
    default:
      return NOTHING;
  }
}

uint32_t Schedule::sleepLeft(uint32_t now) const {
  if (_phase != SLEEP) return 0;
  if (slotted()) {
    // Until the GPS has to start, not until the slot: arriving at the slot
    // with no fix wastes the one moment this unit owns.
    const uint32_t next = nextSampleAt(now);
    if (next == 0) return 0;
    const uint32_t e = epoch(now);
    const uint32_t wake = (next > _lead_s) ? next - _lead_s : 0;
    return wake > e ? (wake - e) * 1000UL : 0;
  }
  const uint32_t due = _interval_s * 1000UL, gone = since(now, _cycle_at);
  return gone >= due ? 0 : due - gone;
}

// ---- the backlog ------------------------------------------------------------

void Backlog::push(const Fix& f) {
  if (_n == TRACKER_BACKLOG_POINTS) {
    _head = (uint16_t)((_head + 1) % TRACKER_BACKLOG_POINTS);
    _n--;
    _lost++;
  }
  _buf[(_head + _n) % TRACKER_BACKLOG_POINTS] = f;
  _n++;
}

const Fix& Backlog::operator[](uint16_t i) const {
  return _buf[(_head + i) % TRACKER_BACKLOG_POINTS];
}

void Backlog::drop(uint16_t n) {
  if (n > _n) n = _n;
  _head = (uint16_t)((_head + n) % TRACKER_BACKLOG_POINTS);
  _n = (uint16_t)(_n - n);
}

// ---- what the base has been given -------------------------------------------

void Reports::sent(const Fix& f, uint32_t ack) {
  if (_report_ack) {
    // The previous report was never answered, so the base does not have
    // it: it joins the path being kept, and one more unanswered report
    // is what tips this unit into looking for a way through.
    //
    // The path, though - not the count. A no-fix report is just as much
    // evidence that the base cannot hear this unit, and must still push it
    // towards searching; it simply has no point to contribute to the trail
    // that gets handed over afterwards.
    if (hasFix(_last)) _backlog.push(_last);
    if (_missed < 255) _missed++;
  }
  _last = f;
  _report_ack = ack;
  _linked = false;
}

Reports::Ack Reports::acked(uint32_t ack) {
  if (ack == 0) return NOT_OURS;
  if (ack == _report_ack) {
    _report_ack = 0;
    _linked = true;
    _missed = 0;                // heard: stop searching, hand the path over
    // A history message still waiting from an earlier cycle has had its
    // chance. Its points never left the backlog, so they simply go again.
    _hist_ack = 0;
    _hist_n = 0;
    return REPORT;
  }
  if (ack == _hist_ack) {
    _backlog.drop(_hist_n);
    _hist_ack = 0;
    _hist_n = 0;
    return HISTORY;
  }
  // Anything else: another copy of an ack already counted - multi-ack sends
  // several - or one for a report that was given up on.
  return NOT_OURS;
}

Reports::Ack Reports::heardInRound() {
  if (_report_ack == 0) return NOT_OURS;
  return acked(_report_ack);
}

bool Reports::historyDue() const {
  return _linked && _hist_ack == 0 && _backlog.size() > 0;
}

uint8_t Reports::nextHistory(char* out, uint32_t now_s) const {
  slimcodec::HistPoint pts[slimcodec::HIST_MAX_POINTS];
  uint8_t n = 0;
  for (; n < slimcodec::HIST_MAX_POINTS && n < _backlog.size(); n++) {
    const Fix& f = _backlog[n];
    pts[n].lat = f.lat;
    pts[n].lon = f.lon;
    pts[n].age_s = now_s >= f.at_s ? now_s - f.at_s : 0;
  }
  return slimcodec::encodeHistory(out, pts, n);
}

}  // namespace tracker

#endif  // WITH_TRACKER_MODE
