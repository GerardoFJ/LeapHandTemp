



struct Options {
  RelativeDirection hand = RD_LEFT;
  bool alternate_ik = true;
  bool legacy_blend = true;
  std::string host = "127.0.0.1";
  int port = 9871;
  double rate_hz = 90.0;
  bool quiet = false;
  bool feedback = false;
  int feedback_port = 9873;
  float engage = 0.30f;
  float release = 0.15f;
  int min_hold_ms = 120;
  int feedback_timeout_ms = 300;
  bool ring_brake = true;
  bool list_brakes = false;
  bool brake_test = false;
};

