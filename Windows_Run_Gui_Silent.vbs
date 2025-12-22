Set WshShell = CreateObject("WScript.Shell")
Dim batPath
batPath = WScript.ScriptFullName
batPath = Left(batPath, Len(batPath) - Len(WScript.ScriptName)) & "run_gui.bat"
WshShell.Run """" & batPath & """", 0, False
Set WshShell = Nothing