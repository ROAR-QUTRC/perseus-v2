builtin_interfaces/Time start_time
builtin_interfaces/Time end_time

# Wake word
string wake_phrase
float32 wake_confidence

# Person tracking
bool person_detected
float32 person_track_duration_sec

# Conversation turns (parallel arrays — index i = turn i)
string[] user_transcripts
string[] llm_responses
float32[] stt_durations_sec
float32[] llm_durations_sec
float32[] tts_durations_sec

# Greeting
float32 greeting_tts_duration_sec

# Totals
uint32 num_turns
float32 total_duration_sec
string end_reason
