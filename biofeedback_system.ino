/**
 * ============================================================================
 * COMPREHENSIVE MULTI-MODAL BIOFEEDBACK SYSTEM v4
 * ============================================================================
 * 
 * Author: Alexander Nichols
 * Institution: Old Dominion University, Department of Physics
 * Course: Physics 303 - Experimental Physics
 * Date: December 2025
 * 
 * OVERVIEW:
 * This system implements a complete biofeedback training device that monitors
 * physiological signals (heart rate and galvanic skin response) and provides
 * real-time multi-modal feedback (visual + auditory) to guide users toward
 * relaxation states. The system demonstrates the principle that providing
 * external feedback about internal states enables conscious regulation of
 * typically involuntary processes.
 * 
 * HARDWARE REQUIREMENTS:
 * - Arduino R4 WiFi (Renesas RA4M1, ARM Cortex-M4, 48 MHz)
 * - Pulse Sensor (optical PPG-based heart rate monitor) on analog pin A0
 * - Grove GSR Sensor (galvanic skin response) on analog pin A1
 * - Four LEDs (Red, Yellow, Green, Blue) on digital pins 2-5
 * - 8Ω 0.5W Speaker on digital pin 6 (PWM-capable)
 * - 220Ω current-limiting resistors for each LED
 * 
 * THEORETICAL FOUNDATION:
 * The system leverages operant conditioning and biofeedback principles:
 * 1. Sensors make normally imperceptible physiological states observable
 * 2. Immediate feedback enables trial-and-error learning
 * 3. Repeated practice strengthens neural pathways for self-regulation
 * 4. Users internalize these skills, eventually self-regulating without feedback
 * 
 * FEEDBACK DESIGN PHILOSOPHY:
 * - Visual: Color progression (Red→Yellow→Green→Blue) with pulse rates
 *   that slow as heart rate decreases, naturally associating calm with
 *   slower rhythms
 * - Auditory: Musical tones descending in pitch as stress decreases,
 *   leveraging cultural associations between lower pitches and calmness
 * - Multi-modal: Combined feedback accommodates different learning styles
 *   and reinforces training through multiple sensory channels
 * 
 * SYSTEM STATES:
 * 1. Initialization: Hardware setup and self-test
 * 2. Calibration: Establishing personalized GSR baseline (5 seconds)
 * 3. Active Monitoring: Real-time biofeedback with continuous feedback
 * 
 * LICENSE: Educational/Open Source
 * ============================================================================
 */

// ============================================================================
// LIBRARY DEPENDENCIES
// ============================================================================

/*
 * PulseSensorPlayground Library Configuration
 * 
 * This library provides robust beat detection and BPM calculation from the
 * optical photoplethysmography (PPG) signal. We enable hardware interrupts
 * for precise timing and reliable beat detection even under varying conditions.
 */
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

/*
 * Pulse Sensor Configuration (Analog Input A0)
 * 
 * The pulse sensor uses an optical method (photoplethysmography) to detect
 * blood volume changes in tissue. An LED illuminates the fingertip, and a
 * photodetector measures reflected light. Each heartbeat causes a pulse of
 * blood flow, temporarily increasing absorption and creating a measurable
 * signal. The sensor outputs an analog voltage (0-1023 ADC counts) that
 * oscillates with each heartbeat.
 */
const int PULSE_INPUT = A0;

/*
 * Beat Detection Threshold
 * 
 * This threshold (550 out of 1024 ADC counts) was empirically determined
 * through testing. It represents the signal level above which a rising edge
 * is considered a heartbeat. Too low causes false positives from noise; too
 * high misses actual beats. This value works well for most users but could
 * be made adaptive in future versions.
 */
const int THRESHOLD = 550;

// Create pulse sensor object for signal processing
PulseSensorPlayground pulseSensor;

/*
 * GSR Sensor Configuration (Analog Input A1)
 * 
 * Galvanic Skin Response (also called Electrodermal Activity) measures
 * electrical conductance of the skin, which varies with sweat gland activity.
 * The eccrine sweat glands are controlled by the sympathetic nervous system
 * and respond to emotional arousal even before perspiration is consciously
 * noticeable. Higher conductance (lower resistance) indicates sympathetic
 * activation - i.e., stress, arousal, or emotional engagement.
 * 
 * The Grove GSR sensor uses a simple voltage divider circuit where the skin
 * acts as a variable resistor. As skin conductance increases, the output
 * voltage increases proportionally.
 */
const int GSR_INPUT = A1;

// ============================================================================
// GSR SIGNAL PROCESSING PARAMETERS
// ============================================================================

/*
 * Exponential Moving Average (EMA) Filter for GSR
 * 
 * GSR signals contain slow-drifting trends and high-frequency noise. We use
 * an exponential moving average (EMA) filter to smooth the signal:
 * 
 *   smoothed = (1 - α) × smoothed_previous + α × raw_measurement
 * 
 * Where α (alpha) is the smoothing factor (0 < α < 1):
 * - Smaller α = more smoothing, slower response
 * - Larger α = less smoothing, faster response
 * 
 * α = 0.05 provides good balance: responsive enough to track genuine changes
 * while rejecting transient noise from hand movements or sensor contact issues.
 */
float gsrSmoothed = 0;           // Current smoothed GSR value
const float gsrAlpha = 0.05f;    // Smoothing factor for GSR signal

/*
 * Personalized GSR Baseline Calibration
 * 
 * GSR varies dramatically between individuals due to:
 * - Skin hydration and thickness
 * - Individual differences in sweat gland density
 * - Baseline autonomic tone
 * 
 * Rather than using absolute GSR values, we establish each user's baseline
 * during a 5-second calibration period. All subsequent measurements are
 * expressed as deviations from this personal baseline, enabling standardized
 * stress level classification regardless of individual physiology.
 */
int gsrBaseline = 0;                              // User's baseline GSR value
bool gsrCalibrated = false;                       // Calibration complete flag
unsigned long calibrationStart = 0;               // Calibration start timestamp
const unsigned long CALIBRATION_TIME = 5000;      // 5 seconds for calibration

// ============================================================================
// LED HARDWARE PINS (Digital Outputs)
// ============================================================================

/*
 * Four-Color LED Array for Visual Feedback
 * 
 * The LED array provides intuitive visual feedback through color progression:
 * RED (highest stress) → YELLOW → GREEN → BLUE (deepest relaxation)
 * 
 * This color scheme leverages universal color associations:
 * - Red: Alertness, danger, "stop"
 * - Yellow: Caution, moderate arousal
 * - Green: Safety, "go," equilibrium
 * - Blue: Calm, tranquility, deep relaxation
 * 
 * LEDs are connected to digital pins through 220Ω current-limiting resistors
 * to protect both the LEDs and the Arduino's output pins (max 8mA per pin
 * for the RA4M1 microcontroller).
 */
const int EXT_LED_RED    = 2;
const int EXT_LED_YELLOW = 3;
const int EXT_LED_GREEN  = 4;
const int EXT_LED_BLUE   = 5;

// ============================================================================
// SPEAKER HARDWARE PIN (PWM Digital Output)
// ============================================================================

/*
 * Audio Feedback via PWM-Driven Speaker
 * 
 * Pin 6 supports Pulse Width Modulation (PWM), enabling tone generation.
 * The Arduino's tone() function generates square waves at specified frequencies,
 * driving an 8Ω 0.5W speaker. Lower frequencies (longer wavelengths) are
 * culturally and psychologically associated with calmness, while higher
 * frequencies signal alertness or urgency.
 */
const int SPEAKER_PIN = 6;

// ============================================================================
// ZONE DEFINITIONS: HEART RATE RANGES AND FEEDBACK PARAMETERS
// ============================================================================

/*
 * Biofeedback Zone Structure
 * 
 * Each zone represents a heart rate range with associated feedback parameters.
 * This graduated approach provides clear targets for users while avoiding
 * sharp boundaries that might be discouraging.
 * 
 * Design Rationale:
 * - minBPM/maxBPM: Define the heart rate range for this zone
 * - name: Human-readable zone identification
 * - pulseSpeed: LED blink interval (ms) - slower for calmer zones
 * - toneFreq: Audio feedback frequency (Hz) - lower for calmer zones
 * - toneDuration: How long each tone plays (ms)
 * 
 * The pulse speed progression creates a visceral sense of slowing down as
 * users achieve lower heart rates, reinforcing the relaxation training.
 */
struct Zone {
  int minBPM;          // Minimum BPM for this zone (inclusive)
  int maxBPM;          // Maximum BPM for this zone (inclusive)
  String name;         // Human-readable zone name
  int pulseSpeed;      // LED pulse interval in milliseconds
  int toneFreq;        // Audio tone frequency in Hertz
  int toneDuration;    // Tone duration in milliseconds
};

/*
 * Seven-Zone Heart Rate Classification System
 * 
 * Zone 0 (Blue Only): 0-59 BPM
 *   - Target state representing deep relaxation
 *   - Parasympathetic dominance, optimal recovery
 *   - Musical note: A2 (220 Hz) - deep, grounding tone
 *   - Slowest pulse rate (1000ms) encourages calmness
 * 
 * Zone 1 (Green-Blue): 60-69 BPM
 *   - Excellent relaxed state
 *   - Musical note: Middle C (262 Hz)
 *   - Transitioning toward optimal
 * 
 * Zone 2 (Green Only): 70-79 BPM
 *   - Good baseline relaxation
 *   - Musical note: D (294 Hz)
 *   - Normal resting heart rate for many adults
 * 
 * Zone 3 (Yellow-Green): 80-89 BPM
 *   - Slightly elevated but manageable
 *   - Musical note: E (330 Hz)
 *   - Light sympathetic activation
 * 
 * Zone 4 (Yellow Only): 90-99 BPM
 *   - Moderately elevated, noticeable stress
 *   - Musical note: F# (370 Hz)
 *   - Clear opportunity for relaxation practice
 * 
 * Zone 5 (Red-Yellow): 100-119 BPM
 *   - Elevated stress, significant sympathetic activation
 *   - Musical note: A (440 Hz) - standard concert pitch
 *   - Breathing exercises strongly recommended
 * 
 * Zone 6 (Red Only): 120-250 BPM
 *   - High stress or physical exertion
 *   - Musical note: High C (523 Hz) - urgent, attention-getting
 *   - Fastest pulse (100ms) signals need for intervention
 * 
 * Note: The musical scale provides naturally harmonious feedback and
 * leverages cultural associations between pitch and emotional state.
 */
Zone zones[] = {
  {0,   59,  "Blue Only",         1000, 220, 100},
  {60,  69,  "Green-Blue",         700, 262, 80},
  {70,  79,  "Green Only",         500, 294, 70},
  {80,  89,  "Yellow-Green",       400, 330, 60},
  {90,  99,  "Yellow Only",        300, 370, 50},
  {100, 119, "Red-Yellow",         200, 440, 40},
  {120, 250, "Red Only",           100, 523, 30}
};

const int NUM_ZONES = 7;  // Total number of defined zones

// ============================================================================
// SYSTEM STATE VARIABLES
// ============================================================================

/*
 * Current Physiological State Tracking
 * 
 * These variables maintain the system's understanding of the user's current
 * state and drive the feedback mechanisms.
 */
int currentBPM = 0;              // Current beats per minute from pulse sensor
int currentZone = -1;            // Current heart rate zone (-1 = not yet determined)
unsigned long lastBlink = 0;     // Timestamp of last LED state change
bool blinkState = false;         // Current LED on/off state

// ============================================================================
// AUDIO CONTROL PARAMETERS
// ============================================================================

/*
 * Audio Feedback Timing and Control
 * 
 * The audio feedback system provides continuous reinforcement without being
 * intrusive. Tones play at intervals matching the LED pulse speed, creating
 * synchronized multi-modal feedback.
 */
unsigned long lastTone = 0;      // Timestamp of last tone
bool enableAudio = true;         // Master audio enable/disable flag

// ============================================================================
// DATA VISUALIZATION AND MONITORING
// ============================================================================

/*
 * Real-Time Serial Plotter Output
 * 
 * The Arduino IDE's Serial Plotter provides real-time visualization of
 * physiological signals and system state. This is invaluable for:
 * - Debugging and system validation
 * - User training and education
 * - Research data collection
 * - Understanding signal quality and artifacts
 * 
 * We output multiple data streams:
 * - Raw and smoothed pulse signal
 * - Current BPM and zone
 * - GSR values (raw and delta from baseline)
 * - Beat detection markers
 */
bool enablePlot = true;                    // Enable/disable serial plotting
const uint16_t plotPeriodMs = 20;         // Plot update interval (50 Hz)
unsigned long lastPlot = 0;                // Last plot timestamp

/*
 * Signal Smoothing for Visualization
 * 
 * We apply exponential moving average filters to create clean, interpretable
 * plots without losing important signal features.
 */
float smoothSignalF = 0;                   // Smoothed pulse signal for plotting
const float signalAlpha = 0.20f;           // Smoothing factor for signal display
float bpmSmoothed = 0;                     // Smoothed BPM for stable display
const float bpmAlpha = 0.10f;              // BPM smoothing factor (slower response)
bool beatFlash = false;                    // Beat marker flag for plotting

// ============================================================================
// SYSTEM INITIALIZATION
// ============================================================================

void setup() {
  /*
   * Serial Communication Setup
   * 
   * High baud rate (115200) enables responsive data streaming for plotting
   * and real-time monitoring without creating communication bottlenecks.
   */
  Serial.begin(115200);

  /*
   * Configure Digital Output Pins
   * 
   * All LEDs and the speaker pin must be explicitly configured as outputs
   * before use. This sets the correct electrical configuration in the
   * microcontroller's GPIO (General Purpose Input/Output) registers.
   */
  pinMode(EXT_LED_RED, OUTPUT);
  pinMode(EXT_LED_YELLOW, OUTPUT);
  pinMode(EXT_LED_GREEN, OUTPUT);
  pinMode(EXT_LED_BLUE, OUTPUT);
  pinMode(SPEAKER_PIN, OUTPUT);

  /*
   * Pulse Sensor Initialization
   * 
   * Configure the PulseSensorPlayground library with our hardware setup:
   * - Tell it which analog pin to read
   * - Set the beat detection threshold
   * - Initialize internal state and interrupt handlers
   */
  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.setThreshold(THRESHOLD);

  /*
   * Attempt Pulse Sensor Initialization
   * 
   * The begin() method returns true if initialization succeeds. If it fails,
   * we enter an infinite error loop with visual and audio warnings to alert
   * the user of hardware problems (disconnected sensor, etc.).
   */
  if (pulseSensor.begin()) {
    // Initialization successful - print welcome messages
    Serial.println("Pulse Sensor initialized!");
    Serial.println("\n=== Comprehensive Biofeedback System V4 ===");
    Serial.println("Sensors: Heart Rate + Galvanic Skin Response");
    Serial.println("Feedback: LED + Audio");
    Serial.println("LED Color Progression: Red -> Yellow -> Green -> Blue");
    Serial.println("Audio: High pitch (alert) -> Low pitch (calm)");
    Serial.println("Goal: Achieve BLUE ONLY (< 60 BPM) with LOW GSR\n");
    
    /*
     * Visual and Audio Startup Sequence
     * 
     * Demonstrate the full feedback progression so users understand what
     * each color/tone represents. This also serves as a hardware self-test,
     * verifying that all LEDs and the speaker are functioning correctly.
     */
    startupSequence();
    
    /*
     * Begin GSR Calibration Phase
     * 
     * Instruct the user to relax and keep still while we establish their
     * personal baseline. This is critical for meaningful stress measurement.
     */
    Serial.println("\n*** GSR CALIBRATION ***");
    Serial.println("Relax and keep fingers still on GSR sensor for 5 seconds...");
    calibrationStart = millis();
    
  } else {
    /*
     * Pulse Sensor Initialization Failed
     * 
     * Enter infinite error loop with alternating LED/audio warnings.
     * This indicates a hardware problem that must be resolved before
     * the system can operate (sensor not connected, wrong pin, etc.).
     */
    Serial.println("Pulse Sensor initialization failed!");
    while (1) {
      allLEDs(true);
      tone(SPEAKER_PIN, 1000);
      delay(500);
      allLEDs(false);
      noTone(SPEAKER_PIN);
      delay(500);
    }
  }
}

// ============================================================================
// MAIN CONTROL LOOP
// ============================================================================

void loop() {
  // ==========================================================================
  // PHASE 1: GSR CALIBRATION (Runs only during first 5 seconds)
  // ==========================================================================
  
  if (!gsrCalibrated) {
    /*
     * GSR Baseline Calibration Process
     * 
     * During the 5-second calibration window, we:
     * 1. Continuously read raw GSR values
     * 2. Apply exponential smoothing to reduce noise
     * 3. At the end, store the smoothed value as the user's baseline
     * 
     * This establishes a personalized reference point against which all
     * future GSR measurements will be compared. The key insight is that
     * absolute GSR values vary wildly between individuals, but changes
     * relative to baseline are meaningful indicators of stress response.
     */
    int gsrRaw = analogRead(GSR_INPUT);
    gsrSmoothed = (1.0f - gsrAlpha) * gsrSmoothed + gsrAlpha * gsrRaw;
    
    // Check if calibration period is complete
    if (millis() - calibrationStart >= CALIBRATION_TIME) {
      gsrBaseline = (int)gsrSmoothed;
      gsrCalibrated = true;
      
      // Inform user that calibration is complete
      Serial.print("GSR Baseline calibrated: ");
      Serial.println(gsrBaseline);
      Serial.println("System ready!\n");
      Serial.println("BPM\tZone\t\tGSR\tStress Level");
      Serial.println("----------------------------------------------------------");
      
      /*
       * Calibration Complete Audio Signal
       * 
       * Play an ascending three-note sequence to signal successful calibration.
       * This provides clear audio feedback that the system is now operational.
       */
      if (enableAudio) {
        tone(SPEAKER_PIN, 262, 150); delay(200);  // Middle C
        tone(SPEAKER_PIN, 330, 150); delay(200);  // E
        tone(SPEAKER_PIN, 392, 150); delay(200);  // G
        noTone(SPEAKER_PIN);
      }
    }
    
    // During calibration, skip the rest of the loop
    return;
  }

  // ==========================================================================
  // PHASE 2: ACTIVE BIOFEEDBACK MONITORING (After calibration complete)
  // ==========================================================================
  
  /*
   * Read and Process GSR Signal
   * 
   * Every loop iteration, we:
   * 1. Read the raw analog value from the GSR sensor (0-1023)
   * 2. Apply exponential smoothing for noise reduction
   * 3. Calculate the difference from the calibrated baseline
   * 
   * gsrDelta represents the current deviation from the user's relaxed state:
   * - Positive values: More stressed than baseline
   * - Negative values: More relaxed than baseline (rare but possible)
   * - Magnitude: Degree of change
   */
  int gsrRaw = analogRead(GSR_INPUT);
  gsrSmoothed = (1.0f - gsrAlpha) * gsrSmoothed + gsrAlpha * gsrRaw;
  int gsrDelta = (int)gsrSmoothed - gsrBaseline;
  
  /*
   * Read Current Heart Rate
   * 
   * The PulseSensorPlayground library continuously analyzes the PPG signal
   * and calculates BPM using inter-beat interval measurements. We simply
   * query the most recent BPM estimate.
   */
  currentBPM = pulseSensor.getBeatsPerMinute();

  /*
   * Beat Detection and Zone Management
   * 
   * sawStartOfBeat() returns true when the library detects a new heartbeat
   * (systolic peak in the PPG waveform). This is the ideal time to:
   * 1. Mark the beat for visualization
   * 2. Check if we've transitioned to a new heart rate zone
   * 3. Trigger zone-specific feedback
   */
  if (pulseSensor.sawStartOfBeat()) {
    beatFlash = true;  // Flag for plotting - creates beat markers in serial plot

    /*
     * Zone Transition Detection and Response
     * 
     * When heart rate moves into a different zone, we need to:
     * 1. Update the current zone state
     * 2. Print updated biofeedback information to serial
     * 3. Play a zone transition sound to alert the user
     * 
     * This ensures users receive immediate feedback about their progress
     * toward (or away from) their relaxation goals.
     */
    int newZone = getZone(currentBPM);
    if (newZone != currentZone) {
      currentZone = newZone;
      printBiofeedbackInfo(gsrDelta);
      
      // Play pleasant harmonic transition sound
      if (enableAudio && currentZone >= 0) {
        playZoneTransitionSound(currentZone);
      }
    }
  }

  /*
   * Smooth BPM for Display
   * 
   * Apply exponential smoothing to BPM values for stable visualization.
   * This prevents jittery displays while remaining responsive to real changes.
   * We only update when we have a valid BPM reading (> 0).
   */
  if (currentBPM > 0) {
    bpmSmoothed = (1.0f - bpmAlpha) * bpmSmoothed + bpmAlpha * currentBPM;
  }

  /*
   * Update Feedback Systems
   * 
   * Once we have a valid zone assignment, continuously update both visual
   * (LED) and auditory (speaker) feedback to guide the user. These functions
   * handle their own timing to ensure appropriate pulse rates and tone intervals.
   */
  if (currentZone >= 0) {
    updateLEDs();
    updateAudio();
  }

  // ==========================================================================
  // REAL-TIME DATA VISUALIZATION
  // ==========================================================================
  
  /*
   * Serial Plotter Output
   * 
   * At regular intervals (20ms = 50 Hz), output formatted data for the
   * Arduino IDE's Serial Plotter. This creates live graphs of:
   * - Pulse signal (raw and smoothed)
   * - Current BPM
   * - Beat markers (spikes at each heartbeat)
   * - Heart rate zone
   * - GSR values (absolute and relative to baseline)
   * 
   * The plotter provides invaluable feedback for:
   * - Verifying proper sensor function
   * - Observing signal quality
   * - Detecting artifacts
   * - Understanding how physiology changes with mental state
   */
  if (enablePlot) {
    unsigned long now = millis();
    if (now - lastPlot >= plotPeriodMs) {
      lastPlot = now;

      // Read and smooth pulse signal for clean visualization
      int raw = analogRead(PULSE_INPUT);
      smoothSignalF = (1.0f - signalAlpha) * smoothSignalF + signalAlpha * raw;
      int smoothSignal = (int)(smoothSignalF + 0.5f);

      // Create beat marker (spike to 1023 when beat detected)
      int beatMarker = beatFlash ? 1023 : 0;
      beatFlash = false;  // Clear the flag after using it

      /*
       * Output Multiple Data Streams
       * 
       * The Serial Plotter interprets tab-separated values with labels
       * (label:value format). Each stream appears as a separate line
       * in the plot, enabling simultaneous visualization of multiple signals.
       */
      Serial.print("PulseSignal:");
      Serial.print(smoothSignal);
      Serial.print("\tBPM:");
      Serial.print((int)(bpmSmoothed + 0.5f));
      Serial.print("\tBeat:");
      Serial.print(beatMarker);
      Serial.print("\tZone:");
      Serial.print(currentZone);
      Serial.print("\tGSR:");
      Serial.print((int)gsrSmoothed);
      Serial.print("\tGSR_Delta:");
      Serial.println(gsrDelta);
    }
  }

  /*
   * Minimal Loop Delay
   * 
   * A 1ms delay prevents the loop from running at maximum speed, which:
   * 1. Reduces unnecessary power consumption
   * 2. Allows interrupt handlers to run smoothly
   * 3. Prevents serial buffer overflow
   * 4. Still provides excellent responsiveness (1000 Hz loop rate)
   */
  delay(1);
}

// ============================================================================
// AUDIO FEEDBACK CONTROL FUNCTION
// ============================================================================

void updateAudio() {
  /*
   * Continuous Audio Feedback Generation
   * 
   * This function generates periodic tones that vary in both frequency and
   * rate based on the current heart rate zone. The audio feedback:
   * 
   * 1. Provides continuous reinforcement (unlike visual feedback which
   *    requires looking at the LEDs)
   * 2. Allows eyes-closed practice, which many find more conducive to
   *    relaxation
   * 3. Reinforces the zone progression through descending pitch
   * 4. Matches the LED pulse rate for multi-modal coherence
   * 
   * Design Notes:
   * - Lower zones = lower frequency tones (more calming)
   * - Lower zones = slower tone intervals (less stimulating)
   * - Shorter tone durations prevent audio fatigue
   */
  
  // Exit early if audio is disabled or zone is invalid
  if (!enableAudio || currentZone < 0) return;
  
  unsigned long currentTime = millis();
  Zone z = zones[currentZone];
  
  /*
   * Periodic Tone Generation
   * 
   * Play a tone at intervals matching the LED pulse speed. This creates
   * synchronized multi-modal feedback that reinforces the training:
   * - Visual (LED pulsing) + Auditory (periodic tones) = stronger learning
   * - Temporal synchrony helps integrate the feedback into a coherent signal
   */
  if (currentTime - lastTone >= (unsigned long)z.pulseSpeed) {
    lastTone = currentTime;
    tone(SPEAKER_PIN, z.toneFreq, z.toneDuration);
  }
}

// ============================================================================
// ZONE TRANSITION AUDIO SIGNALING
// ============================================================================

void playZoneTransitionSound(int zone) {
  /*
   * Harmonic Zone Transition Alert
   * 
   * When moving between zones, play a distinctive two-note sequence to
   * signal the transition. This helps users recognize meaningful changes
   * in their physiological state without having to look at the display.
   * 
   * Musical Theory:
   * The second note is a perfect fifth interval (frequency × 1.25) above
   * the first. Perfect fifths are consonant and pleasant, making the
   * transition sound harmonious rather than jarring. This matters because:
   * - Pleasant sounds don't trigger stress responses
   * - Consonant intervals are easier to process cognitively
   * - Harmonic relationships feel "natural" and non-intrusive
   * 
   * Example: If zone frequency is 440 Hz (A), the second note is 550 Hz (C#)
   * This creates an A-C# interval, which while not precisely a perfect fifth,
   * is close and maintains the harmonic relationship.
   */
  int freq = zones[zone].toneFreq;
  tone(SPEAKER_PIN, freq, 100);           // Base frequency
  delay(120);                              // Brief silence
  tone(SPEAKER_PIN, freq * 1.25, 100);    // Fifth interval
  delay(120);
  noTone(SPEAKER_PIN);                    // Ensure tone stops
}

// ============================================================================
// ZONE CLASSIFICATION FUNCTION
// ============================================================================

int getZone(int bpm) {
  /*
   * Heart Rate Zone Classification
   * 
   * Given a BPM value, determine which of the seven defined zones it falls
   * into. This simple linear search works well given the small number of
   * zones and the infrequency of calls (only when BPM changes significantly).
   * 
   * The function iterates through zones in order and returns the first match.
   * If no zone matches (shouldn't happen given our zone definitions cover
   * 0-250 BPM), we default to the highest zone as a safety measure.
   * 
   * Alternative Implementations:
   * For more zones or higher-performance requirements, binary search or
   * direct calculation would be more efficient. However, readability and
   * maintainability are more important here than raw speed.
   */
  for (int i = 0; i < NUM_ZONES; i++) {
    if (bpm >= zones[i].minBPM && bpm <= zones[i].maxBPM) {
      return i;
    }
  }
  return NUM_ZONES - 1;  // Default to highest zone if no match
}

// ============================================================================
// GSR-BASED STRESS LEVEL CLASSIFICATION
// ============================================================================

String getStressLevel(int gsrDelta) {
  /*
   * Qualitative Stress Assessment from GSR Delta
   * 
   * Translate the numerical GSR delta (deviation from baseline) into
   * human-readable stress level categories. These thresholds were determined
   * empirically through testing and represent typical response patterns:
   * 
   * Very Relaxed (<10): Minimal deviation from baseline
   *   - User is in a calm, settled state
   *   - Skin conductance very close to resting level
   * 
   * Relaxed (<30): Slight elevation
   *   - Normal fluctuations from daily activity
   *   - Not necessarily stressed, just not deeply relaxed
   * 
   * Moderate (<60): Noticeable activation
   *   - Mild stress or engagement
   *   - Beginning of sympathetic response
   * 
   * Elevated (<100): Clear stress response
   *   - Significant sympathetic activation
   *   - User likely aware of feeling stressed
   * 
   * High Stress (≥100): Strong stress response
   *   - Pronounced sympathetic activation
   *   - Conscious stress very likely
   * 
   * Note: These are general guidelines. Individual variation exists, which
   * is why we use baseline-relative rather than absolute measurements.
   */
  if (gsrDelta < 10) return "Very Relaxed";
  else if (gsrDelta < 30) return "Relaxed";
  else if (gsrDelta < 60) return "Moderate";
  else if (gsrDelta < 100) return "Elevated";
  else return "High Stress";
}

// ============================================================================
// LED FEEDBACK CONTROL FUNCTION
// ============================================================================

void updateLEDs() {
  /*
   * Zone-Based LED Control with Pulsing Effect
   * 
   * This function implements the visual feedback system. The LEDs don't just
   * indicate the current zone—they pulse on and off to create a dynamic,
   * attention-holding display that naturally draws the eye and provides
   * temporal rhythm cues.
   * 
   * Pulsing Mechanics:
   * Each zone has a characteristic pulse speed (the interval between state
   * changes). We track the time since the last state change, and when enough
   * time has elapsed, we flip the LED state (on→off or off→on).
   * 
   * Why Pulsing Matters:
   * 1. Dynamic displays are more salient than static ones
   * 2. The pulse rate itself conveys information (slower = calmer)
   * 3. Pulsing creates a meditative visual rhythm
   * 4. The slowing pulse as HR decreases viscerally demonstrates progress
   */
  
  unsigned long currentTime = millis();

  // Check if it's time to toggle the LED state
  if (currentTime - lastBlink >= (unsigned long)zones[currentZone].pulseSpeed) {
    lastBlink = currentTime;
    blinkState = !blinkState;  // Toggle between on and off

    // First, turn off all LEDs to ensure clean state
    allLEDs(false);

    /*
     * Zone-Specific LED Patterns
     * 
     * When LEDs are in their "on" state, illuminate the LED(s) appropriate
     * for the current zone. Some zones use two colors to indicate transitions:
     * - Green-Blue: Transitioning from normal to deeply relaxed
     * - Yellow-Green: Transitioning from elevated to normal
     * - Red-Yellow: Transitioning from high stress to elevated
     * 
     * These transitional zones provide intermediate goals and prevent
     * frustration when users can't immediately achieve the target state.
     */
    if (blinkState) {
      switch (currentZone) {
        case 0:  // Blue Only - GOAL STATE
          digitalWrite(EXT_LED_BLUE, HIGH);
          break;
          
        case 1:  // Green-Blue transition
          digitalWrite(EXT_LED_GREEN, HIGH);
          digitalWrite(EXT_LED_BLUE, HIGH);
          break;
          
        case 2:  // Green Only - Good baseline
          digitalWrite(EXT_LED_GREEN, HIGH);
          break;
          
        case 3:  // Yellow-Green transition
          digitalWrite(EXT_LED_YELLOW, HIGH);
          digitalWrite(EXT_LED_GREEN, HIGH);
          break;
          
        case 4:  // Yellow Only - Moderate stress
          digitalWrite(EXT_LED_YELLOW, HIGH);
          break;
          
        case 5:  // Red-Yellow transition
          digitalWrite(EXT_LED_RED, HIGH);
          digitalWrite(EXT_LED_YELLOW, HIGH);
          break;
          
        case 6:  // Red Only - High stress
          digitalWrite(EXT_LED_RED, HIGH);
          break;
      }
    }
  }
}

// ============================================================================
// LED UTILITY FUNCTION
// ============================================================================

void allLEDs(bool state) {
  /*
   * Bulk LED Control
   * 
   * Simple utility function to set all four LEDs to the same state
   * simultaneously. Used for:
   * - Clearing all LEDs before setting zone-specific patterns
   * - Error indication (all LEDs flashing)
   * - Startup sequence effects
   * 
   * Parameters:
   *   state: true = all LEDs on, false = all LEDs off
   */
  digitalWrite(EXT_LED_RED, state);
  digitalWrite(EXT_LED_YELLOW, state);
  digitalWrite(EXT_LED_GREEN, state);
  digitalWrite(EXT_LED_BLUE, state);
}

// ============================================================================
// USER FEEDBACK AND ENCOURAGEMENT
// ============================================================================

void printBiofeedbackInfo(int gsrDelta) {
  /*
   * Comprehensive Status Reporting with Encouragement
   * 
   * When zones change, print a formatted status line showing:
   * - Current BPM
   * - Zone name (color progression)
   * - GSR delta from baseline
   * - Qualitative stress level
   * - Contextual encouragement/guidance
   * 
   * The encouragement system is crucial for maintaining motivation:
   * - Positive reinforcement for achieving relaxed states
   * - Gentle guidance when stress is elevated
   * - Special recognition for optimal state achievement
   * 
   * This psychological component transforms the system from passive
   * measurement to active coaching, enhancing training effectiveness.
   */
  
  Zone z = zones[currentZone];
  
  // Print core measurements
  Serial.print(currentBPM);
  Serial.print("\t");
  Serial.print(z.name);
  Serial.print("\t\t");
  Serial.print(gsrDelta);
  Serial.print("\t");
  Serial.print(getStressLevel(gsrDelta));

  /*
   * Contextual Feedback Messages
   * 
   * Provide specific feedback based on the combination of heart rate zone
   * and GSR stress level. This multi-modal assessment gives a more complete
   * picture than either measure alone:
   * 
   * Optimal State: Low HR + Low GSR
   *   - Genuine deep relaxation
   *   - Both parasympathetic activation AND low arousal
   *   - The ultimate goal state
   * 
   * Good Progress: Low-moderate HR + Low-moderate GSR
   *   - Moving in the right direction
   *   - Positive reinforcement to maintain motivation
   * 
   * Elevated Stress: High HR OR High GSR
   *   - Guidance to use relaxation techniques
   *   - Reminder of breathing exercises
   */
  if (currentZone == 0 && gsrDelta < 20) {
    // Optimal state achieved - maximum encouragement
    Serial.print(" *** OPTIMAL STATE ***");
  } else if (currentZone <= 1 && gsrDelta < 40) {
    // Very good progress - positive reinforcement
    Serial.print(" - Great work!");
  } else if (currentZone >= 5 || gsrDelta > 80) {
    // Elevated stress detected - gentle guidance
    Serial.print(" - Take deep breaths");
  }

  Serial.println();
}

// ============================================================================
// STARTUP DEMONSTRATION SEQUENCE
// ============================================================================

void startupSequence() {
  /*
   * Educational Startup Sequence
   * 
   * This sequence serves multiple purposes:
   * 1. Hardware Self-Test: Verifies all LEDs and speaker are functioning
   * 2. User Education: Demonstrates what each color/tone represents
   * 3. Goal Visualization: Shows the progression from stress to relaxation
   * 4. System Readiness: Confirms initialization is complete
   * 
   * The sequence progresses from high stress (red, fast, high pitch) to
   * deep relaxation (blue, slow, low pitch), demonstrating the full range
   * of feedback the system can provide. Users see and hear exactly what
   * they should aim for during training.
   * 
   * Timing Strategy:
   * - High-stress zones: Fast transitions (maintain attention)
   * - Low-stress zones: Slow transitions (demonstrate calmness)
   * - This mirrors the actual feedback behavior during use
   */
  
  Serial.println("\nLED + Audio Sequence Demo:");
  Serial.println("Red -> Red+Yellow -> Yellow -> Yellow+Green -> Green -> Green+Blue -> Blue\n");

  // Start with all LEDs off, brief pause for effect
  allLEDs(false);
  delay(500);

  /*
   * Play Ascending Musical Scale
   * 
   * Start with an ascending C major scale as an audio system test and
   * to create a pleasant, attention-getting opening. The scale establishes
   * the audio frequency range and demonstrates the speaker's capability.
   */
  if (enableAudio) {
    int melody[] = {262, 294, 330, 349, 392, 440, 494, 523};  // C major scale
    for (int i = 0; i < 8; i++) {
      tone(SPEAKER_PIN, melody[i], 100);
      delay(120);
    }
    noTone(SPEAKER_PIN);
    delay(300);
  }

  /*
   * Zone 6: Red (High Stress) - Rapid, Urgent
   * 
   * Start with the most stressed state to contrast with the final goal.
   * Fast pulsing and high-pitched tone convey urgency and alertness.
   */
  Serial.print("Red (rapid) -> ");
  for (int i = 0; i < 5; i++) {
    digitalWrite(EXT_LED_RED, HIGH);
    if (enableAudio) tone(SPEAKER_PIN, 523, 50);  // High C
    delay(100);
    digitalWrite(EXT_LED_RED, LOW);
    delay(100);
  }

  /*
   * Zone 5: Red+Yellow (Elevated Stress)
   * 
   * Slightly slower than pure red, showing the beginning of de-escalation.
   * Two-color display indicates a transitional state.
   */
  Serial.print("Red+Yellow -> ");
  for (int i = 0; i < 4; i++) {
    digitalWrite(EXT_LED_RED, HIGH);
    digitalWrite(EXT_LED_YELLOW, HIGH);
    if (enableAudio) tone(SPEAKER_PIN, 440, 75);  // A
    delay(150);
    digitalWrite(EXT_LED_RED, LOW);
    digitalWrite(EXT_LED_YELLOW, LOW);
    delay(150);
  }

  /*
   * Zone 4: Yellow (Moderate Stress)
   * 
   * Noticeably slower pacing and lower tone. Users should begin to feel
   * the psychological shift from urgent to merely elevated.
   */
  Serial.print("Yellow -> ");
  for (int i = 0; i < 3; i++) {
    digitalWrite(EXT_LED_YELLOW, HIGH);
    if (enableAudio) tone(SPEAKER_PIN, 370, 100);  // F#
    delay(200);
    digitalWrite(EXT_LED_YELLOW, LOW);
    delay(200);
  }

  /*
   * Zone 3: Yellow+Green (Slight Elevation)
   * 
   * Introduction of green (associated with "go," safety, equilibrium).
   * Pace continues to slow, approaching comfortable levels.
   */
  Serial.print("Yellow+Green -> ");
  for (int i = 0; i < 3; i++) {
    digitalWrite(EXT_LED_YELLOW, HIGH);
    digitalWrite(EXT_LED_GREEN, HIGH);
    if (enableAudio) tone(SPEAKER_PIN, 330, 125);  // E
    delay(250);
    digitalWrite(EXT_LED_YELLOW, LOW);
    digitalWrite(EXT_LED_GREEN, LOW);
    delay(250);
  }

  /*
   * Zone 2: Green (Good Baseline)
   * 
   * Comfortable, steady state. Many users will spend significant time here.
   * The pacing feels natural and unhurried.
   */
  Serial.print("Green -> ");
  for (int i = 0; i < 3; i++) {
    digitalWrite(EXT_LED_GREEN, HIGH);
    if (enableAudio) tone(SPEAKER_PIN, 294, 175);  // D
    delay(350);
    digitalWrite(EXT_LED_GREEN, LOW);
    delay(350);
  }

  /*
   * Zone 1: Green+Blue (Approaching Goal)
   * 
   * Introduction of blue (associated with calm, tranquility).
   * Very slow pacing creates a meditative rhythm.
   */
  Serial.print("Green+Blue -> ");
  for (int i = 0; i < 2; i++) {
    digitalWrite(EXT_LED_GREEN, HIGH);
    digitalWrite(EXT_LED_BLUE, HIGH);
    if (enableAudio) tone(SPEAKER_PIN, 262, 250);  // Middle C
    delay(500);
    digitalWrite(EXT_LED_GREEN, LOW);
    digitalWrite(EXT_LED_BLUE, LOW);
    delay(500);
  }

  /*
   * Zone 0: Blue (GOAL STATE - Deep Relaxation)
   * 
   * The target state. Slowest pacing and lowest tone create a sense of
   * peaceful achievement. Users should feel the contrast between this and
   * the rapid red pulsing at the start - viscerally demonstrating the
   * journey from stress to tranquility.
   */
  Serial.println("Blue (GOAL)");
  for (int i = 0; i < 2; i++) {
    digitalWrite(EXT_LED_BLUE, HIGH);
    if (enableAudio) tone(SPEAKER_PIN, 220, 350);  // A2 - low, grounding
    delay(700);
    digitalWrite(EXT_LED_BLUE, LOW);
    delay(700);
  }

  // Ensure audio is completely off before proceeding
  noTone(SPEAKER_PIN);
  
  // Final instruction before beginning operation
  Serial.println("\nReady! Place pulse sensor on finger/wrist.");
  delay(1000);
}

/**
 * ============================================================================
 * END OF PROGRAM
 * ============================================================================
 * 
 * USAGE NOTES:
 * 
 * Optimal Sensor Placement:
 * - Pulse Sensor: Fingertip pad (most sensitive) or inside wrist
 *   - Secure with medical tape or elastic band
 *   - Firm contact crucial for signal quality
 *   - Avoid excessive pressure (restricts blood flow)
 * 
 * - GSR Sensor: Two adjacent fingers (index + middle typical)
 *   - Clean, dry skin before measurement
 *   - Maintain consistent pressure
 *   - Minimize movement during sessions
 * 
 * Training Protocol:
 * 1. Sit comfortably in a quiet space
 * 2. Attach sensors and complete calibration
 * 3. Begin with 5-10 minute sessions
 * 4. Focus on slow, deep breathing
 * 5. Use feedback to guide relaxation attempts
 * 6. Practice regularly for best results
 * 
 * Troubleshooting:
 * - Erratic BPM readings: Check pulse sensor contact/placement
 * - No beat detection: Adjust threshold value or sensor position
 * - Drifting GSR: Allow longer equilibration time before calibration
 * - No audio: Check speaker connection and enableAudio flag
 * 
 * Modifications and Extensions:
 * This code is designed to be educational and modifiable. Consider:
 * - Adjusting zone thresholds for personal physiology
 * - Adding data logging to SD card or WiFi transmission
 * - Implementing guided breathing exercises
 * - Creating training programs with progressive difficulty
 * - Adding display output for standalone operation
 * 
 * SCIENTIFIC CONTEXT:
 * 
 * This system implements established biofeedback principles supported by
 * decades of research. Key references include:
 * - Schwartz & Andrasik (2003): Biofeedback: A Practitioner's Guide
 * - Lehrer et al. (2000): Heart rate variability biofeedback
 * - Gevirtz (2013): HRV biofeedback evidence review
 * 
 * For academic context, see the accompanying technical report:
 * "Development of a Multi-Modal Biofeedback System for Stress Reduction"
 * 
 * ACKNOWLEDGMENTS:
 * 
 * This work was completed as part of Physics 303 (Experimental Physics) at
 * Old Dominion University under the guidance of Dr. Charles Sukenik.
 * 
 * Hardware: Arduino R4 WiFi, Pulse Sensor, Grove GSR Sensor
 * Libraries: PulseSensorPlayground
 * 
 * LICENSE: Educational/Open Source
 * Feel free to use, modify, and share with attribution.
 * 
 * ============================================================================
 */

