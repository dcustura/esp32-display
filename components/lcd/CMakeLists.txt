set(srcs
   lcd.cpp
    )

idf_component_register(SRCS ${srcs}
                    REQUIRES scrollbuffer
                    INCLUDE_DIRS .)
