# STM32F769 Audio (wav) Player

This is a project based on STM32F769-Disco board, and implement a basic music player and buttons to control how to play the music. This project uses the STemWin and is add-ons, FreeRTOS is also involved to control the tasks and makes whole project efficiency. This project uses a customized font in SD card, so it is able to display Chinese songs' title and without worry about flash storage. The project right now only support for wav files (but all rates supports).

## How to

Copy /SD/YaHei.xbf to your SD card rootfile, and put *.wav songs to /Music fold. You can use any audio transfer software to make wav files and put them into /Music. 

## Demo video

https://www.youtube.com/watch?v=nAqI61Whyyc&feature=youtu.be

## Todo

* Add other formats (such as MP3 and flac) support.
* Add lyric support.
* Add frequency spectrum