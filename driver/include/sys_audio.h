#ifndef _SYS_AUDIO_H
#define _SYS_AUDIO_H

void AudioVol(unsigned char i);
void MP3WAVplay(char *path);
void MP3WAVplay_exit(void);
void AUDIO_Demo(void);

int wav_init(char *path, int play_time);
void _WAV_Play2(void);

#endif
