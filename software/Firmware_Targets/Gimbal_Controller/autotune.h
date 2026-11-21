#pragma once

#include <simpleFOC.h>
#include <SimpleFOCAutoTune.h>


void runTune(BLDCMotor *motor){
    // auto tune

    if (motor == nullptr)
    {
        Serial.println("Motor not available!");
        return;
    }

    SimpleFOCAutoTune tuner(*motor);

    SimpleFOCAutoTune::Config cfg;

    // Motion testing
    cfg.stepRad = 0.16f;        // Maximum jump angle
    cfg.maxExcursionRad = 0.75f; // 43-degree safety envelope from start
    cfg.maxSpeedRadS = 40.0f;   // Overspeed fault threshold, NOT velocity limit

    cfg.homeMs = 1000;
    cfg.legMs = 3000;

    // Progressive jump angles
    cfg.progressiveStepAngles = true;
    cfg.initialStepRad = 0.16f;
    cfg.stepGrowthFactor = 1.50f;
    cfg.stepAdvanceMinSweeps = 1;
    cfg.stepAdvanceMaxOvershootRad = 0.16f;
    cfg.stepAdvanceMaxSettleFraction = 0.85f;

    // Holding stability
    cfg.holdOffsetRad = 0.035f;
    cfg.holdMs = 3000;
    cfg.offsetSettleMs = 1500;

    // Disturbance recovery
    cfg.kickRad = 0.2f;
    cfg.kickMs = 500;
    cfg.recoveryMs = 3000;

    // Angle output ramp optimization
    cfg.tuneAngleOutputRamp = true;

    // Acceleration limits (rad/s^2)
    cfg.angleRampMinRadS2 = 2.0f;
    cfg.angleRampMaxRadS2 = 200.0f;

    // Initial value if current ramp is disabled or out of bounds
    cfg.angleRampStartRadS2 = 8.0f;

    // Optimization
    cfg.tuneUntilConverged = true;
    cfg.tuneD = true;
    cfg.tuneMotion = true;
    cfg.tuneHolding = true;
    cfg.tuneVelocityFilter = true;

    // Coarse gain exploration
    cfg.coarseToFineSearch = true;
    cfg.coarseDownFactor = 0.750f;
    cfg.coarseUpFactor = 1.250f;
    cfg.searchMinRatio = 0.05f;
    cfg.searchMaxRatio = 10.00f;
    cfg.minScoreImprovementFraction = 0.005f;
    cfg.probeShrinkFactor = 0.50f;
    cfg.minProbeFraction = 0.02f;
    cfg.noImprovementSweepsToStop = 2;

    // Second-stage bracket refinement
    cfg.bracketRefinement = true;
    cfg.refinementBracketFraction = 0.40f;
    cfg.refinementToleranceFraction = 0.02f;
    cfg.refinementMaxIterations = 8;
    cfg.refinementNoImprovementRounds = 2;
    cfg.refinementMinScoreImprovementFraction = 0.0025f;

    // Runtime and trial safety caps (not target trial counts)
    cfg.maxTuneTimeMs = 3600000UL;
    cfg.maxTrials = 1000;

    // Fast motion optimization and holding performance guard
    cfg.optimizeRiseTime = true;
    cfg.optimizeSettlingTime = true;
    cfg.optimizeHoldingJitter = true;
    cfg.preserveMotionPerformance = true;
    cfg.maxRiseTimeRegression = 0.10f;
    cfg.maxSettlingTimeRegression = 0.10f;
    cfg.motionRiseTimeWeight = 0.30f;
    cfg.motionSettlingTimeWeight = 0.60f;
    cfg.motionSettleErrorRad = 0.020f;
    cfg.motionSettleSpeedRadS = 0.10f;
    cfg.motionSettleDwellMs = 100;

    // Adaptive test timing
    cfg.adaptiveTimings = true;
    cfg.timingReductionPerPass = 0.70f;
    cfg.minTimingScale = 0.70f;
    cfg.adaptiveSettledErrorRad = 0.020f;
    cfg.adaptiveSettledSpeedRadS = 0.10f;

    // Overshoot protection
    cfg.automaticGainBackoff = false;

    // Trigger gain reduction before reaching the hard travel envelope
    cfg.violentOvershootRad = 0.4f;

    // Reduce aggressive gains
    cfg.backoffVelocityPFactor = 0.50f;
    cfg.backoffVelocityIFactor = 0.10f;
    cfg.backoffVelocityDFactor = 0.50f;
    cfg.backoffAnglePFactor = 0.35f;

    // Maximum automatic recoveries
    cfg.maxGainBackoffs = 2;

    // Recovery behavior
    cfg.backoffSettleMs = 1500;
    cfg.backoffTimeoutMs = 20000;
    cfg.backoffReturnRateRadS = 0.10f;
    cfg.backoffSettleSpeedRadS = 0.2f;

    // Enable coordinated multi-parameter optimization
    cfg.jointOptimization = true;

    // Explore joint changes before fine bracket refinement
    cfg.jointBeforeRefinement = true;

    // Initial relative gain adjustment
    cfg.jointInitialProbeFraction = 0.25f;

    // Refine down to 2.5% adjustments
    cfg.jointMinProbeFraction = 0.025f;
    cfg.jointProbeShrinkFactor = 0.50f;

    // Continue until joint searches stop improving
    cfg.jointNoImprovementSweepsToStop = 2;

    // Test additional whole-controller directions
    cfg.jointWholeDirections = 8;

    // Require a 0.5% score improvement
    cfg.jointMinScoreImprovementFraction = 0.005f;

    // Independently repeat promising tests
    cfg.jointConfirmImprovements = true;

    // Alternate between joint and bracket refinement
    cfg.jointMaxAlternations = 2;

    // History-assisted mathematical model optimization
    cfg.modelOptimization = true;

    // Store comparable previous trials in RAM
    cfg.modelHistoryLimit = 96;
    cfg.modelMinSamples = 12;
    cfg.modelWarmupSweeps = 2;

    // Model search cycles and candidate budget
    cfg.modelMaxCycles = 2;
    cfg.modelMaxTrialsPerCycle = 24;

    // Adaptive trust region for coordinated parameter changes
    cfg.modelInitialTrustFraction = 0.30f;
    cfg.modelMinTrustFraction = 0.025f;
    cfg.modelMaxTrustFraction = 0.55f;

    // Quadratic model regularization and minimum improvements
    cfg.modelRegularization = 0.10f;
    cfg.modelMinPredictedImprovementFraction = 0.005f;
    cfg.modelMinMeasuredImprovementFraction = 0.005f;

    // Verify promising model predictions with repeat measurements
    cfg.modelConfirmImprovements = true;

    if (!tuner.start(cfg)){
        Serial.println("Tuner failed to start!");
        Serial.println("Check angle mode, enabled motor, sensor, and PID gains.");
        return;
    }

    Serial.println("Starting Tuner");

    uint32_t lastPrint = 0;
    uint16_t lastTrial = 0;
    uint8_t lastBackoff = 0;
    uint8_t lastStepLevel = 255;

    while (tuner.state() == SimpleFOCAutoTune::State::Running){

        motor->loopFOC();
        tuner.update();

        if (tuner.state() != SimpleFOCAutoTune::State::Fault)
        {
            motor->move(tuner.target());
        }

        // Print diagnostics every 500 ms
        if (millis() - lastPrint >= 500)
        {
            lastPrint = millis();

            if (tuner.gainBackoffCount() != lastBackoff)
            {
                lastBackoff = tuner.gainBackoffCount();

                Serial.printf(
                    "OVERSHOOT! Count=%u Overshoot=%.3f rad\n",
                    (unsigned)lastBackoff,
                    tuner.lastOvershootRad());
            }

            if (tuner.stepLevel() != lastStepLevel){
                lastStepLevel = tuner.stepLevel();

                Serial.printf(
                    "JUMP LEVEL=%u | JUMP=%.4f rad (%.2f deg)\n",
                    (unsigned)lastStepLevel,
                    tuner.activeStepRad(),
                    tuner.activeStepRad() * 180.0f / PI);
            }

            if (tuner.jointSearching()) {
                Serial.printf(
                    "JOINT: Pattern=%s | Step=%.3f | "
                    "Sweeps=%u | Confirming=%d\n",
                    tuner.jointPatternName(),
                    tuner.jointProbeFraction(),
                    (unsigned)tuner.completedJointSweeps(),
                    (int)tuner.jointConfirmingCandidate()
                );
            }

            if (tuner.modelSearching()) {
                Serial.printf(
                    "MODEL: HISTORY=%u | CYCLES=%u | TESTS=%u | "
                    "TRUST=%.3f | FIT_RMS=%.4f | NOISE=%.4f | "
                    "PREDICTED=%.6f | CONFIRM=%d\n",
                    (unsigned)tuner.modelHistorySize(),
                    (unsigned)tuner.modelCycleCount(),
                    (unsigned)tuner.modelCandidateCount(),
                    tuner.modelTrustFraction(),
                    tuner.modelFitRmsFraction(),
                    tuner.modelNoiseFraction(),
                    tuner.modelPredictedScore(),
                    (int)tuner.modelConfirmingCandidate()
                );
            }

            Serial.printf(
                "STATE: %s | STAGE: %s | PHASE: %s | SEARCH: %s | "
                "TRIALS: %u | SWEEPS: %u | JUMP: %.4f | "
                "ANGLE: %.4f | TARGET: %.4f | SPEED: %.4f | "
                "P=%.6f | I=%.6f | D=%.8f | AngleP=%.6f | "
                "LPF=%.6f | Angle Ramp: %.4f rad/s^2 | BestScore=%.6f\n",
                tuner.stateName(),
                tuner.stageName(),
                tuner.phaseName(),
                tuner.searchPhaseName(),
                (unsigned)tuner.completedTrials(),
                (unsigned)tuner.completedSweeps(),
                tuner.activeStepRad(),
                motor->shaft_angle,
                tuner.target(),
                motor->shaft_velocity,
                motor->PID_velocity.P,
                motor->PID_velocity.I,
                motor->PID_velocity.D,
                motor->P_angle.P,
                motor->LPF_velocity.Tf,
                motor->P_angle.output_ramp,
                tuner.bestScore());

            if (tuner.refining()){
                Serial.printf(
                    "REFINEMENT: %s | Round=%u | Parameter=%u | "
                    "Iteration=%u | Bracket=[%.6f, %.6f]\n",
                    tuner.refinementParameterName(),
                    (unsigned)tuner.completedRefinementRounds(),
                    (unsigned)tuner.refinementParameter(),
                    (unsigned)tuner.refinementIteration(),
                    tuner.refinementLow(),
                    tuner.refinementHigh());
            }

            if (tuner.completedTrials() != lastTrial){
                lastTrial = tuner.completedTrials();

                const auto &m = tuner.lastMotionMetrics();

                Serial.printf(
                    "MOTION: Valid=%d | Rise=%.3f s | "
                    "Settle=%.3f s | Overshoot=%.4f rad | "
                    "GuardRejections=%u | TimingScale=%.2f\n",
                    (int)m.valid,
                    m.riseTimeS,
                    m.settlingTimeS,
                    m.peakOvershootRad,
                    (unsigned)tuner.motionGuardRejectedTrials(),
                    tuner.timingScale());
            }
        }

        yield();
    }

    // Report final result
    Serial.printf(
        "AUTO-TUNE STOPPED: STATE=%s | REASON=%s | FAULT=%s\n",
        tuner.stateName(),
        tuner.completionReasonName(),
        tuner.faultReasonName());

    Serial.printf(
        "Total trials=%u | Elapsed=%lu ms | "
        "Converged=%d | HoldingSkipped=%d\n",
        (unsigned)tuner.completedTrials(),
        (unsigned long)tuner.elapsedMs(),
        (int)tuner.converged(),
        (int)tuner.holdingSkippedMotionBenchmark());

    Serial.printf(
        "Final jump=%.4f rad | MaxReached=%d | ProgressionStalled=%d\n",
        tuner.activeStepRad(),
        (int)tuner.maxStepReached(),
        (int)tuner.stepProgressionStalled());

    Serial.printf(
        "MODEL SUMMARY: History=%u | Cycles=%u | CandidateTests=%u | "
        "Trust=%.3f | FitRMS=%.4f | Noise=%.4f\n",
        (unsigned)tuner.modelHistorySize(),
        (unsigned)tuner.modelCycleCount(),
        (unsigned)tuner.modelCandidateCount(),
        tuner.modelTrustFraction(),
        tuner.modelFitRmsFraction(),
        tuner.modelNoiseFraction());

    if (tuner.success()){
        uint32_t lastReport = millis() - 5000UL;

        while (true)
        {
            // Keep FOC and position holding running after tuning completes
            motor->loopFOC();
            motor->move(tuner.target());

            if (millis() - lastReport >= 5000)
            {
                lastReport = millis();

                Serial.println("=== TUNING COMPLETE ===");

                Serial.printf("Result: %s\n", tuner.completionReasonName());

                Serial.printf("Velocity P: %.6f\n", motor->PID_velocity.P);
                Serial.printf("Velocity I: %.6f\n", motor->PID_velocity.I);
                Serial.printf("Velocity D: %.6f\n", motor->PID_velocity.D);
                Serial.printf("Angle P:    %.6f\n", motor->P_angle.P);
                Serial.printf("Velocity LPF Tf: %.6f\n", motor->LPF_velocity.Tf);
                Serial.printf("Angle Ramp: %.4f rad/s^2\n", motor->P_angle.output_ramp);

                Serial.printf(
                    "Motion score: %.6f | Holding score: %.6f\n",
                    tuner.motionBestScore(),
                    tuner.holdingBestScore());
            }

            yield();
        }
    }
    else{
        Serial.println("=== TUNING FAULT ===");
        Serial.printf("FAULT REASON: %s\n", tuner.faultReasonName());
        Serial.println("Motor disabled by tuner! Reset only after investigation.");

        while (true)
        {
            delay(100);
        }
    }
}
