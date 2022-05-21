-- premake 5 uses Lua scripting
-- single line comment
-- indentations are arbitary [tabs and spaces are not considered]
-- Always use forward slash '/' : premake auto converts to the appropriate slash 

-- where the files are output to
outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

workspace "Geometry Toolbox"
    
    architecture "x86_64"
    startproject "geom"     -- set startup project
    toolset "v142"          -- toolset v142 = visual studio 2019

    configurations
    {
        "Debug",
        "Release",
    }

    flags
    {
        "MultiProcessorCompile", -- enable multicore compilation
    }

project "geom"
    kind "StaticLib"
    language "C++"
    cppdialect "C++20"
    staticruntime "off"
    warnings "Extra" -- Set warnings level to 4 for this project
    
    -- Engine output directory
    targetdir("%{wks.location}/bin/" .. outputdir .. "/%{prj.name}")
    objdir("%{wks.location}/bin-int/" .. outputdir .. "/%{prj.name}")

    files
    {
        "geom/src/**.h",
        "geom/src/**.hpp",
        "geom/src/**.c",
        "geom/src/**.cpp",
    }

    includedirs
    {
        "src",
    }

    filter "system:windows"
        cppdialect "C++20"
        staticruntime "off"
        systemversion "latest"

    filter "configurations:Debug"
        defines "DEBUG"
        symbols "On"
    
    filter "configurations:Release"
        defines "RELEASE"
        optimize "On"