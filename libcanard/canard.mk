CANARDSRC =	libcanard/canard.c \
			libcanard/canard_driver.c \
			libcanard/dsdl/uavcan/equipment/esc/esc_Status.c \
			libcanard/dsdl/uavcan/equipment/esc/esc_RawCommand.c \
			libcanard/dsdl/uavcan/equipment/esc/esc_RPMCommand.c \
			libcanard/dsdl/uavcan/equipment/actuator/actuator_ArrayCommand.c \
            libcanard/dsdl/uavcan/equipment/actuator/actuator_Command.c \
            libcanard/dsdl/uavcan/equipment/actuator/actuator_Status.c \
            libcanard/dsdl/uavcan/equipment/actuator/actuator_SetLimits.c \
            libcanard/dsdl/uavcan/protocol/protocol_Panic.c \

CANARDINC = libcanard \
			libcanard/dsdl
