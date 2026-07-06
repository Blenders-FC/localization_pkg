# ===========================
# Header
# ===========================

string header
uint8 protocol_version
uint8 packet_number

# ===========================
# Competition
# ===========================

uint8 players_per_team
string competition_type #cambiar a str proxiamemnete

bool stopped

string game_phase #cambiar a str proxiamemnete
string state #cambiar a str proxiamemnete
string set_play #cambiar a str proxiamemnete

bool first_half

uint8 kicking_team

int16 secs_remaining
int16 secondary_time

# ===========================
# Team
# ===========================

uint8 team_number

uint8 field_player_color
uint8 goalkeeper_color

uint8 goalkeeper

uint8 score

uint8 penalty_shot

uint16 single_shots

uint16 message_budget

# ===========================
# Robot
# ===========================

uint8 player_number

string penalty

uint8 secs_till_unpenalised

uint8 cautions

# ===========================
# Robot state (calculado)
# ===========================

string robot_play_state

uint8 robot_play_state_int