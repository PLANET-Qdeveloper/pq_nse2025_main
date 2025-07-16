#ifndef FLOAT_PRINT_H
#define FLOAT_PRINT_H

#define FLOAT_PRINT(f) ((int)(f)), ((int)(((f) < 0 ? -(f) : (f)) * 1000) % 1000)

#endif /* FLOAT_PRINT_H */