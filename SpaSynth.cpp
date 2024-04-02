#include "daisy_seed.h"
#include "daisysp.h"

// Use the daisy namespace to prevent having to type
// daisy:: before all libdaisy functions
using namespace daisy;
using namespace daisysp;

// Declare a DaisySeed object called hardware
DaisySeed hardware;

// Polyphony
const int VOICE_COUNT = 8;
const int STEP_COUNT  = 32;
int       steps[STEP_COUNT][VOICE_COUNT];
int       current_voice = 0;
int       step          = 0;

// Notes/Scales
int       base_note     = 48; // C3
const int minor_scale[] = {0, 2, 3, 5, 7, 8, 10, 12};

// DSP
#define MAX_DELAY static_cast<size_t>(48000 * 2.0f)
DelayLine<float, MAX_DELAY> dl;
Metro                       clock;
Oscillator                  osc[VOICE_COUNT];
AdEnv                       env[VOICE_COUNT];
Svf                         flt[VOICE_COUNT];

// Controls
float mx_knb_1, mx_knb_2, mx_knb_3, mx_knb_4, mx_knb_5, mx_knb_6, mx_knb_7,
    mx_knb_8;
Switch    button1, button2, button3;
const int UPDATE_RATE  = 96;
int       update_count = 0;

// Parameters
float clock_speed, note_spread, main_amp, atk, decay, filter_depth, delay_time,
    delay_feedback;
;

void UpdateControls();
void MapControls();
void TraverseQuintCircle();
void AdvanceSequencer();
void SetEnvControls();
void NextSamples(float &sig);
void SoundGenerator(float &sig);
void Effects(float &sig);

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    float sig;

    update_count++;
    if(update_count == UPDATE_RATE)
    {
        UpdateControls();
        MapControls();
        AdvanceSequencer();
        SetEnvControls();
    }
    update_count %= UPDATE_RATE;


    if(button1.Pressed())
    {
        TraverseQuintCircle();
    }


    clock.SetFreq((clock_speed * 4096) / 60 * STEP_COUNT * UPDATE_RATE);

    sig = 0;
    for(size_t i = 0; i < size; i += 2)
    {
        NextSamples(sig);

        sig = tanh(sig);
        sig *= main_amp;

        out[i]     = sig;
        out[i + 1] = sig;
    }
}


int main(void)
{
    // Configure and Initialize the Daisy Seed
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hardware.Configure();
    hardware.Init();
    hardware.SetAudioBlockSize(1);

    //How many samples we'll output per second
    float samplerate = hardware.AudioSampleRate();

    clock.Init(60 / 60, samplerate);

    //Create an ADC configuration
    AdcChannelConfig adcConfig[2];
    adcConfig[0].InitSingle(seed::A0);
    adcConfig[1].InitMux(seed::A11, 8, seed::D12, seed::D13, seed::D14);

    //Initialize the buttons
    button1.Init(seed::D25);
    button2.Init(seed::D24);
    button3.Init(seed::D23);

    //Set the ADC to use our configuration
    hardware.adc.Init(adcConfig, 2);

    //Set up oscillators
    for(int i = 0; i < VOICE_COUNT; i++)
    {
        osc[i].Init(samplerate);
        osc[i].SetWaveform(osc[i].WAVE_SAW);
        osc[i].SetAmp(1.f);
        osc[i].SetFreq(1000);
    }

    //Set up volume envelopes
    for(int i = 0; i < VOICE_COUNT; i++)
    {
        env[i].Init(samplerate);
        env[i].SetTime(ADENV_SEG_ATTACK, .01);
        env[i].SetTime(ADENV_SEG_DECAY, .4);
        env[i].SetMin(0.0);
        env[i].SetMax(1.f);
        env[i].SetCurve(0);
    }

    //Set up filters
    for(int i = 0; i < VOICE_COUNT; i++)
    {
        flt[i].Init(samplerate);
        flt[i].SetFreq(1000);
        flt[i].SetRes(0.5);
    }

    // Initialize the delay line
    dl.Init();

    //Start the adc
    hardware.adc.Start();

    //Start calling the audio callback
    hardware.StartAudio(AudioCallback);

    // Loop forever
    for(;;) {}
}

void UpdateControls()
{
    // Read Knobs
    mx_knb_1 = hardware.adc.GetMuxFloat(1, 0);
    mx_knb_2 = hardware.adc.GetMuxFloat(1, 1);
    mx_knb_3 = hardware.adc.GetMuxFloat(1, 2);
    mx_knb_4 = hardware.adc.GetMuxFloat(1, 3);
    mx_knb_5 = hardware.adc.GetMuxFloat(1, 4);
    mx_knb_6 = hardware.adc.GetMuxFloat(1, 5);
    mx_knb_7 = hardware.adc.GetMuxFloat(1, 6);
    mx_knb_8 = hardware.adc.GetMuxFloat(1, 7);

    // debounce Buttons
    button1.Debounce();
    button2.Debounce();
    button3.Debounce();
}

void MapControls()
{
    clock_speed    = pow(mx_knb_1, 2);
    note_spread    = pow(mx_knb_2, 2);
    main_amp       = pow(mx_knb_3, 2);
    atk            = pow(mx_knb_4, 2);
    decay          = pow(mx_knb_5, 2);
    filter_depth   = pow(mx_knb_6, 2);
    delay_time     = pow(mx_knb_7, 2);
    delay_feedback = pow(mx_knb_8, 2);
}

void SetEnvControls()
{
    for(int i = 0; i < VOICE_COUNT; i++)
    {
        env[i].SetTime(ADENV_SEG_ATTACK, (atk * 16.0) + 0.01);
        env[i].SetTime(ADENV_SEG_DECAY, (decay * 32.0) + 0.01);
    }
}

void AdvanceSequencer()
{
    if(clock.Process())
    {
        // if step == 0 repopulate steps
        if(step == 0)
        {
            int rand_note           = rand() / ((RAND_MAX + 1u) / 8);
            int current_base_note   = base_note + minor_scale[rand_note];
            steps[0][current_voice] = current_base_note;
            current_voice++;
            current_voice %= VOICE_COUNT;

            int max_offset = floor((STEP_COUNT - 1) * note_spread) + 1;

            int rand_step = rand() / ((RAND_MAX + 1u) / max_offset);
            steps[rand_step][current_voice]
                = current_base_note + minor_scale[(rand_note + 2) % 8];
            current_voice++;
            current_voice %= VOICE_COUNT;

            rand_step = rand() / ((RAND_MAX + 1u) / max_offset);
            steps[rand_step][current_voice]
                = current_base_note + minor_scale[(rand_note + 4) % 8];
            current_voice++;
            current_voice %= VOICE_COUNT;
        }

        // trigger notes
        for(int i = 0; i < VOICE_COUNT; i++)
        {
            int current_note = steps[step][i];
            if(current_note > 0)
            {
                osc[i].SetFreq(mtof(current_note));
                env[i].Trigger();
                steps[step][i] = 0;
            }
        }

        // advance step
        step++;
        step %= STEP_COUNT;
    }
}

void NextSamples(float &sig)
{
    SoundGenerator(sig);
    Effects(sig);
}

void SoundGenerator(float &sig)
{
    for(int i = 0; i < VOICE_COUNT; i++)
    {
        float env_out = env[i].Process();
        osc[i].SetAmp(env_out);
        float current_sig = osc[i].Process() * 0.5f;
        flt[i].SetFreq(200 + (env_out * filter_depth * 4000));
        flt[i].Process(current_sig);
        sig += flt[i].Low();
    }
}

void Effects(float &sig)
{
    dl.SetDelay(delay_time * (MAX_DELAY - 0.1f));
    float delay_sig = sig + dl.Read();
    dl.Write(delay_sig * delay_feedback);
    sig = delay_sig;
}

void TraverseQuintCircle()
{
    base_note += 7;
    base_note %= 127;
}
