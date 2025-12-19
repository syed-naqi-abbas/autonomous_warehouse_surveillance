/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo front_leftPID, front_rightPID, back_leftPID, back_rightPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   front_leftPID.TargetTicksPerFrame = 0.0;
   front_leftPID.Encoder = readEncoder(FRONT_LEFT);
   front_leftPID.PrevEnc = front_leftPID.Encoder;
   front_leftPID.output = 0;
   front_leftPID.PrevInput = 0;
   front_leftPID.ITerm = 0;

   front_rightPID.TargetTicksPerFrame = 0.0;
   front_rightPID.Encoder = readEncoder(FRONT_RIGHT);
   front_rightPID.PrevEnc = front_rightPID.Encoder;
   front_rightPID.output = 0;
   front_rightPID.PrevInput = 0;
   front_rightPID.ITerm = 0;

   back_leftPID.TargetTicksPerFrame = 0.0;
   back_leftPID.Encoder = readEncoder(BACK_LEFT);
   back_leftPID.PrevEnc = back_leftPID.Encoder;
   back_leftPID.output = 0;
   back_leftPID.PrevInput = 0;
   back_leftPID.ITerm = 0;

   back_rightPID.TargetTicksPerFrame = 0.0;
   back_rightPID.Encoder = readEncoder(BACK_RIGHT);
   back_rightPID.PrevEnc = back_rightPID.Encoder;
   back_rightPID.output = 0;
   back_rightPID.PrevInput = 0;
   back_rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  front_leftPID.Encoder = readEncoder(FRONT_LEFT);
  front_rightPID.Encoder = readEncoder(FRONT_RIGHT);
  back_leftPID.Encoder = readEncoder(BACK_LEFT);
  back_rightPID.Encoder = readEncoder(BACK_RIGHT);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (front_leftPID.PrevInput != 0 || front_rightPID.PrevInput != 0 || back_leftPID.PrevInput != 0 || back_rightPID.PrevInput != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&front_rightPID);
  doPID(&front_leftPID);
  doPID(&back_rightPID);
  doPID(&back_leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(front_leftPID.output, front_rightPID.output, back_leftPID.output, back_rightPID.output);
}