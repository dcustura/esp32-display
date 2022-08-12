set(srcs
   lcd_main.cpp
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS . )
