@rem Gradle wrapper script for Windows

@if "%DEBUG%"=="" @echo off
@rem Set local scope for variables
setlocal

set APP_NAME=Gradle
set APP_BASE_NAME=%~n0
set APP_HOME=%~dp0

set CLASSPATH=%APP_HOME%\gradle\wrapper\gradle-wrapper.jar

%JAVA_HOME%\bin\java.exe -classpath "%CLASSPATH%" org.gradle.wrapper.GradleWrapperMain %*

@rem End local scope for variables
endlocal
