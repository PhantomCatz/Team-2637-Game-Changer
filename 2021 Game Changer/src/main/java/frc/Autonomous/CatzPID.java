/** This file, 'c', is a HAL component that provides Proportional/
    Integeral/Derivative control loops.  It is a realtime component.
    
    It supports a maximum of 16 PID loops.
    
    The number of pid components is set by the module parameter 'num_chan='
    when the component is insmod'ed.  Alternatively, use the
    names= specifier and a list of unique names separated by commas.
    The names= and num_chan= specifiers are mutually exclusive.
    
    In this documentation, it is assumed that we are discussing position
    loops.  However this component can be used to implement other loops
    such as speed loops, torch height control, and others.
    
    Each loop has a number of pins and parameters, whose names begin
    with 'x.', where 'x' is the channel number.  Channel numbers
    start at zero.
    
    The three most important pins are 'command', 'feedback', and
    'output'.  For a position loop, 'command' and 'feedback' are
    in position units.  For a linear axis, this could be inches,
    mm, metres, or whatever is relavent.  Likewise, for a angular
    axis, it could be degrees, radians, etc.  The units of the
    'output' pin represent the change needed to make the feedback
    match the command.  As such, for a position loop 'Output' is
    a velocity, in inches/sec, mm/sec, degrees/sec, etc.
    
    Each loop has several other pins as well.  'error' is equal to
    'command' minus 'feedback'.  'enable' is a bit that enables
    the loop.  If 'enable' is false, all integrators are reset,
    and the output is forced to zero.  If 'enable' is true, the
    loop operates normally.
    
    The PID gains, limits, and other 'tunable' features of the
    loop are implemented as parameters.  These are as follows:
    
    Pgain	Proportional gain
    Igain	Integral gain
    Dgain	Derivative gain
    bias	Constant offset on output
    FF0		Zeroth order Feedforward gain
    FF1		First order Feedforward gain
    FF2		Second order Feedforward gain
    FF3		Third order Feedforward gain
    deadband	Amount of error that will be ignored
    maxerror	Limit on error
    maxerrorI	Limit on error integrator
    maxerrorD	Limit on error differentiator
    maxcmdD	Limit on command differentiator
    maxcmdDD	Limit on command 2nd derivative
    maxcmdDDD	Limit on command 3rd derivative
    maxoutput	Limit on output value
    
    All of the limits (max____) are implemented such that if the
    parameter value is zero, there is no limit.
    
    A number of internal values which may be useful for testing
    and tuning are also available as parameters.  To avoid cluttering
    the parameter list, these are only exported if "debug=1" is
    specified on the insmod command line.
    
    errorI	Integral of error
    errorD	Derivative of error
    commandD	Derivative of the command
    commandDD	2nd derivative of the command
    commandDDD	3rd derivative of the command
    
    The PID loop calculations are as follows (see the code for
    all the nitty gritty details):
    
    error = command - feedback
    if ( abs(error) < deadband ) then error = 0
    limit error to +/- maxerror
    errorI += error * period
    limit errorI to +/- maxerrorI
    errorD = (error - previouserror) / period
    limit errorD to +/- maxerrorD
    commandD = (command - previouscommand) / period
    limit commandD to +/- maxcmdD
    commandDD = (commandD - previouscommandD) / period
    limit commandDD to +/- maxcmdDD
    commandDDD = (commandDD - previouscommandDD) / period
    limit commandDDD to +/- maxcmdDDD
    output = bias + error * Pgain + errorI * Igain +
             errorD * Dgain + command * FF0 + commandD * FF1 +
             commandDD * FF2 + commandDDD * FF3
    limit output to +/- maxoutput
    
    This component exports one function called 'x.do-pid-calcs'
    for each PID loop.  This allows loops to be included in different
    threads and execute at different rates.
*/

/* Note to reader: 
 * The text above was copied from the original c file from the linuxcnc code. 
 * I have copied them here to serve as an explanation of how the original code
 * worked in case it is needed as reference.
 * As such, some statements in the text above may not be true for our code.
 * 
 * The names of the variables/methods/classes were also copied based on the original c code
 * so they may not follow the coding conventions we have for our team.
 * 
 * The calc_pid method below is a direct translation of the equivalent method from the original c 
 * file. The other methods such as toInt and charIsTrue were made to compensate for the way
 * the code was originally written in c, as c does not have booleans.
 * 
 * The original struct called hal_pid_t was converted into a number of
 * private variables each accessible through the class's getters and setters.
 * A basic PID object can be created using three parameters for pgain, igain, and dgain.
*/

package frc.Autonomous;

public class CatzPID {
	private boolean enable = true;
	private float command;	/* pin: commanded value */
    private float commandvds;	/* pin: commanded derivative dummysig */
    private float commandv;	/* pin: commanded derivative value */
    private float feedback;	/* pin: feedback value */
    private float feedbackvds;	/* pin: feedback derivative dummysig */
    private float feedbackv;	/* pin: feedback derivative value */
    private float error;		/* pin: command - feedback */
    private float deadband;	/* pin: deadband */
    private float maxerror;	/* pin: limit for error */
    private float maxerror_i;	/* pin: limit for integrated error */
    private float maxerror_d;	/* pin: limit for differentiated error */
    private float maxcmd_d;	/* pin: limit for differentiated cmd */
    private float maxcmd_dd;	/* pin: limit for 2nd derivative of cmd */
    private float maxcmd_ddd;	/* pin: limit for 3rd derivative of cmd */
    private float error_i;	/* opt. pin: integrated error */
    private double prev_error;		/* previous error for differentiator */
    private float error_d;	/* opt. pin: differentiated error */
    private double prev_cmd;		/* previous command for differentiator */
    private double prev_fb;		/* previous feedback for differentiator */
    private double limit_state;		/* +1 or -1 if in limit, else 0.0 */
    private float cmd_d;		/* opt. pin: differentiated command */
    private float cmd_dd;	/* opt. pin: 2nd derivative of command */
    private float cmd_ddd;	/* opt. pin: 3rd derivative of command */
    private float bias;		/* param: steady state offset */
    private float pgain;		/* pin: proportional gain */
    private float igain;		/* pin: integral gain */
    private float dgain;		/* pin: derivative gain */
    private float ff0gain;	/* pin: feedforward proportional */
    private float ff1gain;	/* pin: feedforward derivative */
    private float ff2gain;	/* pin: feedforward 2nd derivative */
    private float ff3gain;	/* pin: feedforward 3rd derivative */
    private float maxoutput;	/* pin: limit for PID output */
    private float output;	/* pin: the output value */
    private boolean saturated;	/* pin: TRUE when the output is saturated */
    private float saturated_s;  /* pin: the time the output has been saturated */
    private int saturated_count;
			       				/* pin: the time the output has been saturated */
    private boolean index_enable;   /* pin: to monitor for step changes that would
                                 		otherwise screw up FF */
    private boolean error_previous_target; /* pin: measure error as new position vs previous command, to match motion's ideas */
    private char prev_ie;
    
    
    public CatzPID(float pgain, float igain, float dgain) 
    {
    	this.pgain = pgain;
    	this.igain = igain;
    	this.dgain = dgain;
    }
    
    public CatzPID() 
    {
    	
    }
	
    
    //log command, feedback, gains
    //pass in period as seconds
	public void calc_pid(double periodfp) 
	{
		double tmp1, tmp2, tmp3, command, feedback;
		int enable;
		double periodrecip;
		
		/* precalculate some timing constants */
	    periodrecip = 1.0 / periodfp;
	    
	    /* get the enable bit */
	    enable = toInt(isEnable());
	    
	    /* read the command and feedback only once */
	    command = getCommand();
	    feedback = getFeedback();
	    
	    /* calculate the error */
	    if((!(charIsTrue(getPrev_ie()) && !isIndex_enable())) && (isError_previous_target())) 
	    {
	    	// the user requests ferror against prev_cmd, and we can honor
	        // that request because we haven't just had an index reset that
	        // screwed it up.  Otherwise, if we did just have an index
	        // reset, we will present an unwanted ferror proportional to
	        // velocity for this period, but velocity is usually very small
	        // during index search.
		    tmp1 = getPrev_cmd() - feedback;
		} 
	    else
		{
		    tmp1 = command - feedback;
		}
	    
	    /* store error to error pin */
	    setError((float)tmp1);
	    
	    /* apply error limits */
	    if (getMaxerror() != 0.0) 
	    {
	    	if (tmp1 > getMaxerror()) 
	    	{
	    	    tmp1 = getMaxerror();
	    	} 
	    	else if (tmp1 < -getMaxerror()) 
	    	{
	    	    tmp1 = -getMaxerror();
	    	}
	    }
	    
	    /* apply the deadband */
	    if (tmp1 > getDeadband()) 
	    {
	    	tmp1 -= getDeadband();
	    } 
	    else if (tmp1 < -getDeadband()) 
	    {
	    	tmp1 += getDeadband();
	    } 
	    else 
	    {
	    	tmp1 = 0;
	    }
	    
	    /* do integrator calcs only if enabled */
	    if (enable != 0) {
	    	
			/* if output is in limit, don't let integrator wind up */
			if ( ( tmp1 * getLimit_state() ) <= 0.0 ) 
			{
			    /* compute integral term */
			    setError_i((float)(getError_i() + tmp1 * periodfp));
			}
			
			/* apply integrator limits */
			if (getMaxerror_i() != 0.0) 
			{
			    if (getError_i() > getMaxerror_i()) 
			    {
			    	setError_i(getMaxerror_i());
			    } 
			    else if (getError_i() < -getMaxerror_i()) 
			    {
			    	setError_i(-getMaxerror_i());
			    }
			}
	    } 
	    else 
	    {
	    	/* not enabled, reset integrator */
	    	setError_i(0);
	    }
	    
	    /* compute command and feedback derivatives to dummysigs */
	    if(!(charIsTrue(getPrev_ie()) && !isIndex_enable())) 
	    {
	        setCommandvds((float)((command - getPrev_cmd()) * periodrecip));
	        setFeedbackvds((float)((feedback - getPrev_fb()) * periodrecip));
	    }
	    
	    /* and calculate derivative term as difference of derivatives */
	    setError_d(getCommandv() - getFeedbackv());
	    setPrev_error(tmp1);
	    
	    /* apply derivative limits */
	    if (getMaxerror_d() != 0.0) 
	    {
	    	if (getError_d() > getMaxerror_d()) 
	    	{
	    		setError_d(getMaxerror_d());
	    	} 
	    	else if (getError_d() < -getMaxerror_d()) 
	    	{
	    		setError_d(-getMaxerror_d());
	    	}
	    }
	    
	    /* save old value for 2nd derivative calc later */
	    tmp2 = getCmd_d();
	    setCmd_d(getCommandv());

	    // save ie for next time
	    setPrev_ie((char)toInt(isIndex_enable()));

	    setPrev_cmd(command);
	    setPrev_fb(feedback);
	    
	    /* apply derivative limits */
	    if (getMaxcmd_d() != 0.0) 
	    {
	    	if (getCmd_d() > getMaxcmd_d()) 
	    	{
	    		setCmd_d(getMaxcmd_d());
	    	} 
	    	else if (getCmd_d() < -getMaxcmd_d()) 
	    	{
	    		setCmd_d(-getMaxcmd_d());
	    	}
	    }
	    
	    /* save old value for 3rd derivative calc later */
	    tmp3 = getCmd_dd();
	    
	    /* calculate 2nd derivative of command */
	    setCmd_dd((float)((getCmd_d() - tmp2) * periodrecip));
	    
	    /* apply 2nd derivative limits */
	    if (getMaxcmd_dd() != 0.0) 
	    {
	    	if (getCmd_dd() > getMaxcmd_dd()) 
	    	{
	    		setCmd_dd(getMaxcmd_dd());
	    	} 
	    	else if (getCmd_dd() < -getMaxcmd_dd()) 
	    	{
	    		setCmd_dd(-getMaxcmd_dd());
	    	}
	    }
	    
	    /* calculate 3rd derivative of command */
	    setCmd_ddd((float)((getCmd_dd() - tmp3) * periodrecip));
	    
	    /* apply 3rd derivative limits */
	    if (getMaxcmd_ddd() != 0.0) 
	    {
	    	if (getCmd_ddd() > getMaxcmd_ddd()) 
	    	{
	    		setCmd_ddd(getMaxcmd_ddd());
	    	} 
	    	else if (getCmd_ddd() < -getMaxcmd_ddd()) 
	    	{
	    		setCmd_ddd(-getMaxcmd_ddd());
	    	}
	    }
	    
	    /* do output calcs only if enabled */
	    if (enable != 0) {
	    	/* calculate the output value */
			tmp1 =
			    getBias() + getPgain() * tmp1 + getIgain() * getError_i() +
			    getDgain() * getError_d();
			
			tmp1 += command * getFf0gain() + getCmd_d() * getFf1gain() +
			    getCmd_dd() * getFf2gain() + getCmd_ddd() * getFf3gain();
			
			/* apply output limits */
			if (getMaxoutput() != 0.0) 
			{
			    if (tmp1 > getMaxoutput()) 
			    {
			    	tmp1 = getMaxoutput();
			    	setLimit_state(1.0);
			    } 
			    else if (tmp1 < -getMaxoutput()) 
			    {
			    	tmp1 = -getMaxoutput();
			    	setLimit_state(-1.0);
			    } 
			    else 
			    {
			    	setLimit_state(0.0);
			    }
			}
	    } 
	    else 
	    {
	    	/* not enabled, force output to zero */
	    	tmp1 = 0.0;
	    	setLimit_state(0.0);
	    }
	    
	    /* write final output value to output pin */
	    setOutput((float)tmp1);
	    
	    /* set 'saturated' outputs */
	    if(getLimit_state() != 0.0) 
	    { 
	        setSaturated(true);
	        setSaturated_s((float)(getSaturated_s() + periodfp));
	        if(getSaturated_count() != 2147483647)
	            setSaturated_count(getSaturated_count() + 1);
	    } 
	    else 
	    {
	        setSaturated(false);
	        setSaturated_s(0);
	        setSaturated_count(0);
	    }
	    /* done */
	}//end of pid_calc method

	//converts a boolean into an integer of 1 or 0
	private int toInt(boolean b) {
		int val = b ? 1 : 0;
		return val;
	}

	//converts a character variable into a boolean
	//if the character has a value of 0, returns false. Otherwise, returns true 
	private boolean charIsTrue(char c) {
		return c != 0;
	}
	
	//getters and setters for all the private variables of this class
	public boolean isEnable() {
		return enable;
	}
	public void setEnable(boolean enable) {
		this.enable = enable;
	}
	public float getCommand() {
		return command;
	}
	public void setCommand(float command) {
		this.command = command;
	}
	public float getCommandvds() {
		return commandvds;
	}
	public void setCommandvds(float commandvds) {
		this.commandvds = commandvds;
	}
	public float getCommandv() {
		return commandv;
	}
	public void setCommandv(float commandv) {
		this.commandv = commandv;
	}
	public float getFeedback() {
		return feedback;
	}
	public void setFeedback(float feedback) {
		this.feedback = feedback;
	}
	public float getFeedbackvds() {
		return feedbackvds;
	}
	public void setFeedbackvds(float feedbackvds) {
		this.feedbackvds = feedbackvds;
	}
	public float getFeedbackv() {
		return feedbackv;
	}
	public void setFeedbackv(float feedbackv) {
		this.feedbackv = feedbackv;
	}
	public float getError() {
		return error;
	}
	public void setError(float error) {
		this.error = error;
	}
	public float getDeadband() {
		return deadband;
	}
	public void setDeadband(float deadband) {
		this.deadband = deadband;
	}
	public float getMaxerror() {
		return maxerror;
	}
	public void setMaxerror(float maxerror) {
		this.maxerror = maxerror;
	}
	public float getMaxerror_i() {
		return maxerror_i;
	}
	public void setMaxerror_i(float maxerror_i) {
		this.maxerror_i = maxerror_i;
	}
	public float getMaxerror_d() {
		return maxerror_d;
	}
	public void setMaxerror_d(float maxerror_d) {
		this.maxerror_d = maxerror_d;
	}
	public float getMaxcmd_d() {
		return maxcmd_d;
	}
	public void setMaxcmd_d(float maxcmd_d) {
		this.maxcmd_d = maxcmd_d;
	}
	public float getMaxcmd_dd() {
		return maxcmd_dd;
	}
	public void setMaxcmd_dd(float maxcmd_dd) {
		this.maxcmd_dd = maxcmd_dd;
	}
	public float getMaxcmd_ddd() {
		return maxcmd_ddd;
	}
	public void setMaxcmd_ddd(float maxcmd_ddd) {
		this.maxcmd_ddd = maxcmd_ddd;
	}
	public float getError_i() {
		return error_i;
	}
	public void setError_i(float error_i) {
		this.error_i = error_i;
	}
	public double getPrev_error() {
		return prev_error;
	}
	public void setPrev_error(double prev_error) {
		this.prev_error = prev_error;
	}
	public float getError_d() {
		return error_d;
	}
	public void setError_d(float error_d) {
		this.error_d = error_d;
	}
	public double getPrev_cmd() {
		return prev_cmd;
	}
	public void setPrev_cmd(double prev_cmd) {
		this.prev_cmd = prev_cmd;
	}
	public double getPrev_fb() {
		return prev_fb;
	}
	public void setPrev_fb(double prev_fb) {
		this.prev_fb = prev_fb;
	}
	public double getLimit_state() {
		return limit_state;
	}
	public void setLimit_state(double limit_state) {
		this.limit_state = limit_state;
	}
	public float getCmd_d() {
		return cmd_d;
	}
	public void setCmd_d(float cmd_d) {
		this.cmd_d = cmd_d;
	}
	public float getCmd_dd() {
		return cmd_dd;
	}
	public void setCmd_dd(float cmd_dd) {
		this.cmd_dd = cmd_dd;
	}
	public float getCmd_ddd() {
		return cmd_ddd;
	}
	public void setCmd_ddd(float cmd_ddd) {
		this.cmd_ddd = cmd_ddd;
	}
	public float getBias() {
		return bias;
	}
	public void setBias(float bias) {
		this.bias = bias;
	}
	public float getPgain() {
		return pgain;
	}
	public void setPgain(float pgain) {
		this.pgain = pgain;
	}
	public float getIgain() {
		return igain;
	}
	public void setIgain(float igain) {
		this.igain = igain;
	}
	public float getDgain() {
		return dgain;
	}
	public void setDgain(float dgain) {
		this.dgain = dgain;
	}
	public float getFf0gain() {
		return ff0gain;
	}
	public void setFf0gain(float ff0gain) {
		this.ff0gain = ff0gain;
	}
	public float getFf1gain() {
		return ff1gain;
	}
	public void setFf1gain(float ff1gain) {
		this.ff1gain = ff1gain;
	}
	public float getFf2gain() {
		return ff2gain;
	}
	public void setFf2gain(float ff2gain) {
		this.ff2gain = ff2gain;
	}
	public float getFf3gain() {
		return ff3gain;
	}
	public void setFf3gain(float ff3gain) {
		this.ff3gain = ff3gain;
	}
	public float getMaxoutput() {
		return maxoutput;
	}
	public void setMaxoutput(float maxoutput) {
		this.maxoutput = maxoutput;
	}
	public float getOutput() {
		return output;
	}
	public void setOutput(float output) {
		this.output = output;
	}
	public boolean isSaturated() {
		return saturated;
	}
	public void setSaturated(boolean saturated) {
		this.saturated = saturated;
	}
	public float getSaturated_s() {
		return saturated_s;
	}
	public void setSaturated_s(float saturated_s) {
		this.saturated_s = saturated_s;
	}
	public int getSaturated_count() {
		return saturated_count;
	}
	public void setSaturated_count(int saturated_count) {
		this.saturated_count = saturated_count;
	}
	public boolean isIndex_enable() {
		return index_enable;
	}
	public void setIndex_enable(boolean index_enable) {
		this.index_enable = index_enable;
	}
	public boolean isError_previous_target() {
		return error_previous_target;
	}
	public void setError_previous_target(boolean error_previous_target) {
		this.error_previous_target = error_previous_target;
	}
	public char getPrev_ie() {
		return prev_ie;
	}
	public void setPrev_ie(char prev_ie) {
		this.prev_ie = prev_ie;
	}
}
