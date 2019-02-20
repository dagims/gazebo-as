include_directories( "${CMAKE_SOURCE_DIR}/deps/steamaudio/include" )

set (HAVE_STEAMAUDIO OFF PARENT_SCOPE)

# TODO check if libav is available and that the audio decoder will be built into 
#      the util package

if (WIN32)
	if (MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 8)
		set (GAZEBO_STEAM_LIBRARY "${CMAKE_SOURCE_DIR}/deps/steamaudio/lib/Windows/x64/phonon.lib" PARENT_SCOPE )
	else(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 8)
		set (GAZEBO_STEAM_LIBRARY "${CMAKE_SOURCE_DIR}/deps/steamaudio/lib/Windows/x86/phonon.lib" PARENT_SCOPE )
	endif(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 8)

elseif(APPLE)
	set (GAZEBO_STEAM_LIBRARY "${CMAKE_SOURCE_DIR}/deps/steamaudio/lib/OSX/libphonon.dylib" PARENT_SCOPE )

elseif(UNIX)
	if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
		set (HAVE_STEAMAUDIO ON PARENT_SCOPE)
		set (GAZEBO_STEAM_LIBRARY "${CMAKE_SOURCE_DIR}/deps/steamaudio/lib/Linux/x64/libphonon.so" PARENT_SCOPE )
	else(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
		set (GAZEBO_STEAM_LIBRARY "${CMAKE_SOURCE_DIR}/deps/steamaudio/lib/Linux/x86/libphonon.so" PARENT_SCOPE )
	endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
endif()

