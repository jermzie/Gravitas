
// Comment out to disable debug logs
#define DEBUG

#ifdef DEBUG
#define CLOGI(fmt, ...) printf(fmt "\n", ##__VA_ARGS__)
#define CLOGE(fmt, ...) fprintf(stderr, "ERROR: " fmt "\n", ##__VA_ARGS__)
#define CLOGW(fmt, ...) fprintf(stderr, "WARNING: " fmt "\n", ##__VA_ARGS__)
#else
#define CLOGI(fmt, ...) ((void)0)
#define CLOGE(fmt, ...) ((void)0)
#define CLOGW(fmt, ...) ((void)0)
#endif
