# Skip all standard library headers
skip -gfi /usr/include/c++/*/*
skip -gfi /usr/include/c++/*
# Skip internal compiler headers
skip -gfi /usr/lib/gcc/*

# Additional debugging options
set print pretty on
set print array-indexes on
set print object on
set demangle-style gnu-v3