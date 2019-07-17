## MCUFRIEND_kbv fork for ILI9341 displays

Removed:
* Support for other controllers.
* Sanity checking on function arguments (coordinates and such) - save yourself some CPU cycles by not passing garbage values.

### Benchmarks versus the original full library

Using the slightly modified version of the official graphictest_kbv example: https://github.com/VioletGiraffe/graphics-benchmark-arduino/blob/master/Template.ino

Stock Arduino Due board and toolchain.

Compiler flags: -std=c++17 -O2 -fno-threadsafe-statics
* speed test: 1.91 sec vs. 2.10 (original)
* flash size used: 39520 vs. 44880 bytes
* RAM used: 3432 vs. 3436 bytes

Compiler flags: -std=c++17 -O3 -fno-threadsafe-statics
* speed test: 1.82 sec vs. 2.01 (original)
* flash size used: 48436 vs. 57400 bytes
* RAM used: 3432 vs. 3436 bytes
