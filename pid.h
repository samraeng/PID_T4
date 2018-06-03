 
typedef struct { 

  signed int16 dState,     // Last position input 
               iState;     // Integrator state 

  signed int16 iMax, 
               iMin;       // Maximum and minimum allowable integrator state 

  signed int16 iGain,      // integral gain 
               pGain,      // proportional gain 
               dGain;      // derivative gain 
} SPid; 

SPid plantPID; 

signed int16 plantCommand, position, drive; 

//PID controller code 
signed int16 UpdatePID(SPid *pid, signed int16 error, signed int16 position) 
{ 
   signed int16 pTerm, 
                dTerm, 
                iTerm; 

   // calculate the proportional term 
   pTerm =  pid->pGain * error;   // calculate the proportional term 

   // calculate the integral state with appropriate limiting 
   pid->iState += error; 

   if ( pid->iState > pid->iMax){ 
       pid->iState = pid->iMax; 
   } 

   else if ( pid->iState <  pid->iMin){ 
      pid->iState = pid->iMin; 
   } 

   iTerm = pid->iGain * pid->iState;  // calculate the integral term 

   dTerm = pid->dGain * ( pid->dState - position); 

   pid->dState = position; 

   return (pTerm + dTerm + iTerm); 
}
