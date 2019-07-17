## MCUFRIEND_kbv fork for ILI9341 displays

Removed:
* Support for other controllers.
* Sanity checking on function arguments (coordinates and such) - save yourself some CPU cycles by not passing garbage values.

### Benchmarks versus the original full library

Using the graphictest_kbv example (https://github.com/prenticedavid/MCUFRIEND_kbv/tree/master/examples/graphictest_kbv).

Stock Arduino Due board and toolchain.

Compiler flags: -std=c++17 -O2 -fno-threadsafe-statics
* speed test: 1.93 sec vs. 2.07 (original)
* flash size used: 38232 vs. 43608 bytes
* RAM used: 3428 vs. 3436 bytes

Compiler flags: -std=c++17 -O3 -fno-threadsafe-statics
* speed test: 1.93 sec vs. 2.05 (original)
* flash size used: 47148 vs. 56128 bytes
* RAM used: 3428 vs. 3436 bytes