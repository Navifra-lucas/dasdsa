@startuml

[*] --> idle
idle --> run : Events::onTaskAdd [isValid] / SendGoal
run --> run : Events::onTaskAdd  [isValid] / SendGoals
run --> idle : Events::onTaskDone [isComplete]
run --> alarm : Events::onAlarm
run --> pause : Events::onPause / Pause
run --> idle : Events::onCancel / Cancle
alarm --> pause : Events::onClearAlarm / Pause
pause --> run : Events::onResume / Resume
pause --> idle : Events::onCancel / Cancle

@enduml