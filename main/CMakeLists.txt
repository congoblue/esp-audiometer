idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS ".")

# With CONFIG_LV_ATTRIBUTE_FAST_MEM_USE_IRAM, LVGL 9 puts IRAM_ATTR on both the
# declaration and the definition of some draw functions. Each IRAM_ATTR creates
# a new section name (.iram1.N), so GCC warns that the second one is ignored.
# The function still goes into IRAM, so the warning is harmless, but ESP-IDF
# builds with -Werror. Silence it for the LVGL library only, as Waveshare's own
# examples do.
idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)
target_compile_options(${LVGL_LIB} PRIVATE -Wno-attributes)
