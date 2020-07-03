
project('USBTMC Demo',
	['c'],
	default_options : [
		'c_std=c11', 'cpp_std=c++11',
		'buildtype=debugoptimized',
		# 'werror=true'
	],
	# license: 'MIT',
	meson_version: '>=0.52.0', # NOTE: Still a WIP, I bump it up whenever I use a feature that warns about my required version versus what I have set
	version: '0'
)

# This is a dirty trick for now - I will bring FreeRTOS in as an external dependency at some point
freertos_src_path =  './../../src/FreeRTOSv10.2.1_191129/FreeRTOS/Source'
freertos_port_path = freertos_src_path / 'portable/GCC/ARM_CM4F'

freertos_srcs = files(
	freertos_src_path / 'list.c',
	freertos_src_path / 'queue.c',
	freertos_src_path / 'tasks.c',
	# freertos_src_path / 'event_groups.c',
	# freertos_src_path / 'timers.c',
	freertos_src_path / 'portable/MemMang/heap_4.c',
	freertos_src_path / 'timers.c',
	freertos_port_path / 'port.c'
)

freertos_incdirs = include_directories(
	freertos_src_path / 'include',
	freertos_port_path
)

incdirs = [
	include_directories('src/include'),
	# CMSIS Core M
	include_directories('src/ext/CMSIS/CMSIS/Core/Include'),
	# CMSIS Device - STM32F4
	include_directories('src/ext/CMSIS-Device-STM32F4/Include'),
	freertos_incdirs
]

stm32f4_device = 'STM32F415xx'

c_args = ['-ffreestanding', '-D', stm32f4_device]

c_link_args = ['-nostartfiles','-T','../src/stm32f4.ld']

srcs = [
	'src/main.c',
	freertos_srcs
]

tgt_elf = executable(
	'usbtmc_demo.elf',
	sources: [ srcs ],
	# dependencies: ssconfig.dependencies(),
	include_directories: incdirs,
	c_args : c_args,
	link_args : c_link_args,
	# implicit_include_directories: false # I think
)
