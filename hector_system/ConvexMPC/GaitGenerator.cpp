#include "GaitGenerator.h"


Gait::Gait(int mpc_horizon, Vec2<int> dsp_durations, Vec2<int> ssp_durations, double dt, double dt_mpc)
: _mpc_horizon(mpc_horizon), _dsp_durations(dsp_durations.array()), _ssp_durations(ssp_durations.array()), _dt(dt), _dt_mpc(dt_mpc) 
{

    //********Gait Cycle Design for Bipeds********//
    // All the unit is control timestep (1 control step ticks 1 control_dt)
    // L:  |    Stance     |    Swing   |St |
    // R:  |   Swing   |       Stance       |
    //Time:|   SSP,L   |DSP|   SSP,R    |DSP|
    // Gait Cycle Length = 2*DSP + SSP,L + SSP,R
    //********Gait Cycle Design for Bipeds********//

    //For standing, set DSP = 0.5*MPC_horizon (actially it can be arbitrary value), SSP,L = 0, SSP,R = 0
    // Phase is within 0 to 1, you can control the how fast each gait ticks with stepping_frequency (1.0 means no speedup)

    // _mpc_table = std::make_shared<int[]>(2*_mpc_horizon); // NOT supported in CXX14
    _mpc_table = std::make_unique<int[]>(2*_mpc_horizon); 
    _gait_cycle_length = _dsp_durations.sum() + _ssp_durations.sum();

    _stance_durations << ssp_durations[0]+dsp_durations.sum(), ssp_durations[1]+dsp_durations.sum(); // left foot, right foot
    _stance_durations_phase = _stance_durations.cast<double>() / (double)_gait_cycle_length; 

    _swing_durations << ssp_durations[1], ssp_durations[0]; // left foot, right foot
    _swing_durations_phase = _swing_durations.cast<double>() / (double)_gait_cycle_length;

    _ssp_durations_phase = _ssp_durations.cast<double>() / (double)_gait_cycle_length;
    _dsp_durations_phase = _dsp_durations.cast<double>() / (double)_gait_cycle_length;

    _swing = _swing_durations; 
    _stance = _stance_durations;

    
    _swing_durations_sec = _swing_durations.cast<double>() * _dt_mpc;
    _stance_durations_sec = _stance_durations.cast<double>() * _dt_mpc;
    gait_durations_sec = _gait_cycle_length * _dt_mpc;

}

Gait::~Gait() = default;


void Gait::update_parameter(Vec2<int> dsp_durations, Vec2<int> ssp_durations)
{
    _dsp_durations = dsp_durations.array();
    _ssp_durations = ssp_durations.array();
    _gait_cycle_length = _dsp_durations.sum() + _ssp_durations.sum();

    _stance_durations << ssp_durations[0]+dsp_durations.sum(), ssp_durations[1]+dsp_durations.sum(); // left foot, right foot
    _stance_durations_phase = _stance_durations.cast<double>() / (double)_gait_cycle_length; 

    _swing_durations << ssp_durations[1], ssp_durations[0]; // left foot, right foot
    _swing_durations_phase = _swing_durations.cast<double>() / (double)_gait_cycle_length;

    _ssp_durations_phase = _ssp_durations.cast<double>() / (double)_gait_cycle_length;
    _dsp_durations_phase = _dsp_durations.cast<double>() / (double)_gait_cycle_length;

    _swing = _swing_durations;
    _stance = _stance_durations;

    _swing_durations_sec = _swing_durations.cast<double>() * _dt_mpc;
    _stance_durations_sec = _stance_durations.cast<double>() * _dt_mpc;
    gait_durations_sec = _gait_cycle_length * _dt_mpc;

    reset();
}

void Gait::updatePhase()
{
    // update in real-time
    _gait_phase += _dt/gait_durations_sec; 
    if (_gait_phase > 1.0)
    {
        _gait_phase -= 1.0;
    }
}

void Gait::updateSamplingTime(double dt_mpc){
  _dt_mpc = dt_mpc;
  _swing_durations_sec = _swing_durations.cast<double>() * _dt_mpc;
  _stance_durations_sec = _stance_durations.cast<double>() * _dt_mpc;
  gait_durations_sec = _gait_cycle_length * _dt_mpc;
}

// Subphase is phase of each foot in contact/swing duration

Vec2<double> Gait::getContactSubPhase() 
{

  Array2d contact_sub_phase = {0, 0};

  //For Left Foot: assumes that stance of the left foot starts from (SSP,L + DSP + SSP,R) timestep in the gait cycle
  if (_gait_phase < _ssp_durations_phase[0] + _dsp_durations_phase[0])
  {
    contact_sub_phase[0] = (double)(_gait_phase)/(double)(_ssp_durations_phase[0] + _dsp_durations_phase[0]);
  }
  else if (_gait_phase < _ssp_durations_phase[0] + _dsp_durations_phase[0] + _ssp_durations_phase[1])
  {
    contact_sub_phase[0] = -1; // swing
  }
  else
  {
    contact_sub_phase[0] = (double)(_gait_phase - (_ssp_durations_phase[0] + _dsp_durations_phase[0] + _ssp_durations_phase[1]))/(double)(_dsp_durations_phase[1]);
  }

  //For Right Foot: assumes that stance of the right foot starts from (SSP,L) timestep in the gait cycle
  if (_gait_phase < _ssp_durations_phase[0])
  {
    contact_sub_phase[1] = -1; //swing
  }
  else
  {
    contact_sub_phase[1] = (double)(_gait_phase - _ssp_durations_phase[0])/(double)(_dsp_durations_phase[0]+_ssp_durations_phase[1]+_dsp_durations_phase[1]);
  }

  return contact_sub_phase.matrix();
}

Vec2<double> Gait::getSwingSubPhase(){

  Array2d swing_sub_phase = {0, 0};

  //For Left Foot
  
  if (_ssp_durations_phase[0] + _dsp_durations_phase[0] <= _gait_phase && _gait_phase < _ssp_durations_phase[0] + _dsp_durations_phase[0] + _ssp_durations_phase[1])
  {
    swing_sub_phase[0] = (double)(_gait_phase - (_ssp_durations_phase[0] + _dsp_durations_phase[0])) / (double)(_ssp_durations_phase[1]);
  }
  else
  {
    swing_sub_phase[0] = -1; // stance
  }


  //For Right Foot
  if (_gait_phase < _ssp_durations_phase[0])
  {
    swing_sub_phase[1] = (double)(_gait_phase) / (double)(_ssp_durations_phase[0]);
  }
  else
  {
    swing_sub_phase[1] = -1; // stance
  }
  
  return swing_sub_phase.matrix();
    
}


int *Gait::mpc_gait(){
  // compute contact sequence during mpc horizon
  for (int tMPC = 0; tMPC < _mpc_horizon; tMPC++)
  {
    int gait_time_step_from_phase = (int)(_gait_phase * (double)_gait_cycle_length);
    int gait_time_step_mpc_forward = (gait_time_step_from_phase + tMPC) % _gait_cycle_length;

    if (gait_time_step_mpc_forward < _ssp_durations[0])
    {
      _mpc_table.get()[tMPC * 2 + 0] = 1; // left foot: stance
      _mpc_table.get()[tMPC * 2 + 1] = 0; // right foot: swing
    }
    else if (_ssp_durations[0] <= gait_time_step_mpc_forward && gait_time_step_mpc_forward < _ssp_durations[0] + _dsp_durations[0])
    {
      _mpc_table.get()[tMPC * 2 + 0] = 1; // left foot: stance
      _mpc_table.get()[tMPC * 2 + 1] = 1; // right foot: stance
    }
    else if (_ssp_durations[0] + _dsp_durations[0] <= gait_time_step_mpc_forward && gait_time_step_mpc_forward < _ssp_durations[0] + _dsp_durations[0] + _ssp_durations[1])
    {
      _mpc_table.get()[tMPC * 2 + 0] = 0; // left foot: swing
      _mpc_table.get()[tMPC * 2 + 1] = 1; // right foot: stance
    }
    else{
      _mpc_table.get()[tMPC * 2 + 0] = 1; // left foot: stance
      _mpc_table.get()[tMPC * 2 + 1] = 1; // right foot: stance
    }

  }

  return _mpc_table.get();
}