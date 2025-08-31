idf_component_register(
    SRCS "ld2420.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_common            # public: headers visible to dependents
    PRIV_REQUIRES freertos esp_timer log  # private: only used inside ld2420.c
)
