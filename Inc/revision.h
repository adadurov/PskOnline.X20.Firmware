
#define BUILD_DATE __DATE__ " " __TIME__

// To prevent a bug when transferring > 64 bytes of data,
// the length of this string is limited to 22 bytes.
// FIXME: If you want to include something long, like the below text:
// #define REVISION_INFO "0.1.0-alpha_cac294f00bb592fcdb55c2ef424f3708606e4409"
// you have to fix the bug.
#define REVISION_INFO "0.1.0"
//#define REVISION_INFO "0.1.0-alpha-a8f44830c601c15961c19599cc0a3d8c0bbfc8e8"

