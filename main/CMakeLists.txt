set(srcs
    "spi_master_example_main.c"
    "ascii_8x13.h"
    "ascii_8x12.h"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")
