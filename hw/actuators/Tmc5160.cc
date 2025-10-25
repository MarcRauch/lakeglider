#include "hw/actuators/Tmc5160.hh"

#include <cmath>

#include "hw/Pins.hh"
#include "utils/Consts.hh"

namespace {
// Consts
constexpr uint32_t MICRO_STEPS = 256;
constexpr uint32_t STEPS_PER_REVOLUTION = 2048;
constexpr double MICRO_STEPS_PER_RAD =
    static_cast<double>(MICRO_STEPS * STEPS_PER_REVOLUTION) / (2. * gl::utils::consts::PI);

// Registers
union Gconf {
  struct {
    uint32_t recalibrate : 1;
    uint32_t faststandstill : 1;
    uint32_t enPwmMode : 1;
    uint32_t multistepFilt : 1;
    uint32_t shaft : 1;
    uint32_t diag0Error : 1;
    uint32_t diag0Otpw : 1;
    uint32_t diag0Step : 1;
    uint32_t diag1Dir : 1;
    uint32_t diag1Index : 1;
    uint32_t diag1Onstate : 1;
    uint32_t diag1StepsSkipped : 1;
    uint32_t diag0IntPushpull : 1;
    uint32_t diag1PoscompPushpull : 1;
    uint32_t smallHysteris : 1;
    uint32_t stopEnable : 1;
    uint32_t directMode : 1;
    uint32_t testMode : 1;
  };
  uint32_t bytes = 0;
};

union Gstat {
  struct {
    uint32_t reset : 1;
    uint32_t drvErr : 1;
    uint32_t uvCp : 1;
  };
  uint32_t bytes = 0;
};

union GlobalScaler {
  struct {
    uint32_t scaler : 8;
  };
  uint32_t bytes = 0;
};

union IholdIrun {
  struct {
    uint32_t iHold : 8;
    uint32_t iRun : 8;
    uint32_t iHoldDelay : 4;
  };
  uint32_t bytes = 0;
};

union Rampmode {
  struct {
    uint32_t rampmode : 2;
  };
  uint32_t bytes = 0;
};

union Xactual {
  struct {
    uint32_t xActual : 32;
  };
  uint32_t bytes = 0;
};

union Vactual {
  struct {
    uint32_t vActual : 24;
  };
  uint32_t bytes = 0;
};

union VStart {
  struct {
    uint32_t vStart : 18;
  };
  uint32_t bytes = 0;
};

union VStop {
  struct {
    uint32_t vStop : 18;
  };
  uint32_t bytes = 0;
};

union V1 {
  struct {
    uint32_t v1 : 20;
  };
  uint32_t bytes = 0;
};

union Amax {
  struct {
    uint32_t aMax : 16;
  };
  uint32_t bytes = 0;
};

union Vmax {
  struct {
    uint32_t vMax : 23;
  };
  uint32_t bytes = 0;
};

union Dmax {
  struct {
    uint32_t dMax : 16;
  };
  uint32_t bytes = 0;
};

union D1 {
  struct {
    uint32_t d1 : 16;
  };
  uint32_t bytes = 0;
};

union Xtarget {
  struct {
    uint32_t xTarget : 32;
  };
  uint32_t bytes = 0;
};

union SwMode {
  struct {
    uint32_t stopLEnable : 1;
    uint32_t stopREnable : 1;
    uint32_t polStopL : 1;
    uint32_t polStopR : 1;
    uint32_t swapLr : 1;
    uint32_t latchLActive : 1;
    uint32_t latchLInactive : 1;
    uint32_t latchRActive : 1;
    uint32_t latchRInactive : 1;
    uint32_t enLatchEncoder : 1;
    uint32_t sgStop : 1;
    uint32_t enSoftstop : 1;
  };
  uint32_t bytes = 0;
};

union RampStat {
  struct {
    uint32_t statusStopL : 1;
    uint32_t statusStopR : 1;
    uint32_t statusLatchL : 1;
    uint32_t statusLatchR : 1;
    uint32_t eventStopL : 1;
    uint32_t eventStopR : 1;
    uint32_t eventStopSg : 1;
    uint32_t eventPosReached : 1;
    uint32_t velocityReached : 1;
    uint32_t positionReached : 1;
    uint32_t vZero : 1;
    uint32_t tZerowaitActive : 1;
    uint32_t secondMove : 1;
    uint32_t statusSg : 1;
  };
  uint32_t bytes = 0;
};

union ChopConf {
  struct {
    uint32_t toff : 4;
    uint32_t hstrt : 3;
    uint32_t hend : 4;
    uint32_t fd3 : 1;
    uint32_t disfdcc : 1;
    uint32_t reserved1 : 1;
    uint32_t chm : 1;
    uint32_t tbl : 2;
    uint32_t reserved2 : 1;
    uint32_t vhigh : 2;
    uint32_t tpfd : 4;
    uint32_t mres : 4;
    uint32_t intpol : 1;
    uint32_t dedge : 1;
    uint32_t diss2g : 1;
    uint32_t diss2vs : 1;
  };
  uint32_t bytes = 0;
};

union PwmConf {
  struct {
    uint32_t pwmOfs : 8;
    uint32_t pwmBrad : 8;
    uint32_t pwmFreq : 2;
    uint32_t pwmAutoscale : 1;
    uint32_t pwmAutograd : 1;
    uint32_t freewheel : 2;
    uint32_t reserved : 2;
    uint32_t pwmReg : 4;
    uint32_t pwmLim : 4;
  };
  uint32_t bytes = 0;
};

// Helper functions

void currentLimits(const gl::hw::Tmc5160::Config& config, IholdIrun* iholdIrun, GlobalScaler* globalScaler) {
  constexpr double vFs_v = 0.325;
  constexpr double rSens_ohm = 0.075;

  const auto calcIRms = [&](uint32_t scale, uint32_t cs) {
    return static_cast<double>(scale * (cs + 1) * vFs_v) / static_cast<double>(256 * 32 * rSens_ohm * sqrt(2));
  };

  uint32_t globalScale = 256;
  uint32_t iRun = 31;
  for (; globalScale > 31 && calcIRms(globalScale, iRun) > config.iRun_a; globalScale--) {}
  globalScale = std::min(globalScale + 1, static_cast<uint32_t>(256));

  for (; iRun > 0 && calcIRms(globalScale, iRun) > config.iRun_a; iRun--) {}

  if (globalScale >= 256) {
    globalScale = 0;
  }
  iholdIrun->iRun = iRun;
  iholdIrun->iHold = static_cast<uint32_t>(static_cast<double>(iRun) * config.iHoldFactor);
  iholdIrun->iHoldDelay = 7;
  globalScaler->scaler = globalScale;
}

uint32_t calculateVelocity(double vel_radps) {
  constexpr double t_s = 1.39810133333;  // 2^24/f_clk
  const double microStepsPerS = vel_radps * MICRO_STEPS_PER_RAD;
  return static_cast<uint32_t>(microStepsPerS / t_s);
}

uint32_t calculateAcceleration(double acc_radps2) {
  constexpr double t_s2 = 2.19902325555;  // 2^41/f_clk^2
  double microstepsPerS2 = acc_radps2 * MICRO_STEPS_PER_RAD;
  return static_cast<uint32_t>(microstepsPerS2 / t_s2);
}

}  // namespace

namespace gl {
namespace hw {

Tmc5160::Tmc5160(ISpi* spi, utils::IClock* clock, const Config& config) : config(config), spi(spi), clock(clock) {
  // Initialize device
  writeRegister(Register::GSTAT, Gstat{.reset = 1, .uvCp = 1});
  writeRegister(Register::GCONF, Gconf{.enPwmMode = 1, .shaft = config.invertDirection});
  IholdIrun iholdIrun;
  GlobalScaler globalscaler;
  currentLimits(config, &iholdIrun, &globalscaler);
  writeRegister(Register::GLOBAL_SCALER, globalscaler);
  writeRegister(Register::IHOLD_IRUN, iholdIrun);
  writeRegister(Register::PWM_CONF,
                PwmConf{.pwmOfs = 30, .pwmFreq = 1, .pwmAutoscale = 1, .pwmAutograd = 1, .pwmReg = 4, .pwmLim = 12});
  writeRegister(Register::CHOP_CONF, ChopConf{.toff = 5, .hstrt = 4, .tbl = 2});
  writeRegister(Register::RAMPMODE, Rampmode{.rampmode = 0});
  writeRegister(Register::VSTART, VStart{.vStart = 0});
  writeRegister(Register::VSTOP, VStop{.vStop = 10});
  writeRegister(Register::V1, V1{.v1 = 0});
  writeRegister(Register::D1, D1{.d1 = 100});
  writeRegister(Register::VMAX, Vmax{.vMax = calculateVelocity(config.vel_radps)});
  writeRegister(Register::AMAX, Amax{.aMax = calculateAcceleration(config.acc_radps2)});
  writeRegister(Register::DMAX, Dmax{.dMax = calculateAcceleration(config.acc_radps2)});
  writeRegister(Register::SW_MODE, SwMode{.stopLEnable = 1});
  clock->wait(utils::Time::sec(2));
}

void Tmc5160::home() {
  writeRegister(Register::RAMPMODE, Rampmode{.bytes = 2});
  while (!readRegister<RampStat>(Register::RAMP_STAT).statusStopL) {
    clock->wait(utils::Time::msec(100));
  }
  writeRegister(Register::XACTUAL, Xactual{.xActual = 0});
  writeRegister(Register::XTARGET, Xtarget{.xTarget = 0});
  writeRegister(Register::RAMPMODE, Rampmode{.rampmode = 0});
}

void Tmc5160::moveTo(double pos) {
  pos = std::max(std::min(pos, 1.), 0.);
  const double maxMicroSteps = config.maxRotations * STEPS_PER_REVOLUTION * MICRO_STEPS;
  writeRegister(Register::XTARGET, Xtarget{.xTarget = static_cast<uint32_t>(pos * maxMicroSteps)});
}

}  // namespace hw
}  // namespace gl