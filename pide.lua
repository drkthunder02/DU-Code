local pide = {}
pide.__index = pide;

-- Constructor
local function new(p, i, d, tTime, minLimit, maxLimit)

	return setmetatable({
		-- Controller Gains
		Kp = p,
		Ki = i,
		Kd = d,
		
		-- Derivative low-pass filter time constant
		tau = 0,
		
		-- Output limits
		limMin = minLimit or 0.0,
		limMax = maxLimit or 100.0,
		
		-- Sample time in seconds
		tTime = tTime or 1,
		
		-- Controller memory
		integrator = 0.0,
		err = 0.0,
		prevError = 0.0,
		differentiator = 0.0,
		prevMeasurement = 0.0,
		output = 0.0		
	}, pide)
end

function pide:update(setpoint, measurement)
	-- Error signal
	err = setpoint - measurement
	
	
	-- Proportion Term
	local proportional = self.Kp * err
	
	-- Integral Term
	self.integrator = self.integrator + 0.5 * self.Ki * self.tTime * (err + self.prevError)
	
	-- Anti-Windup integrator clamping
	if self.integrator > self.limMax then
		self.integrator = self.limMax
	end
	
	if self.integrator < self.limMin then
		self.integrator = self.limMin
	end
	

	-- Derivative Term
	self.differentiator = -(2.0 * self.Kd * (measurement - self.prevMeasurement) + (2 * self.tau - self.tTime) * self.differentiator) / (2.0 * self.tau + self.tTime)
	
	-- Compute output and apply limits
	self.out = proportional + self.integrator + self.differentiator

	if self.out > self.limMax then
		self.out = self.limMax
	end
	
	if self.out < self.limMin then
		self.out = self.limMin
	end
	
	-- Store error and measurement for later use
	self.prevError = err
	self.prevMeasurement = measurement
	return self.out
end

function pide:setTau(tau)
	self.tau = tau
end

function pide:setTime(t)
	self.tTime = t
end

function pide:setLimits(minL, maxL)
	self.minLimit = minL
	self.maxLimit = maxL
end

-- the module
return setmetatable(
	{
		new = new, update = update, setTau = setTau, setTime = setTime, setLimits = setLimits
	}, {
		__call = function(_, ...) return new(...) end
	}
)