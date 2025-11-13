#include "FSM/FSMState.h"

std::unique_ptr<unitree::robot::g1::publisher::LowCmd> FSMState::lowcmd = nullptr;
std::shared_ptr<unitree::robot::g1::subscription::LowState> FSMState::lowstate = nullptr;
std::shared_ptr<unitree::robot::g1::subscription::SportModeState> FSMState::sportmodestate = nullptr;
int FSMState::nq = 29;