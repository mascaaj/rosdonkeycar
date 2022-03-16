# Servo Connections & i2cpwm board:

dc motor propel:
- pin set 11, 
- servo address 12
- Neutral at 333

steering servo:
- pin set 8
- servo address 9
- neutral at 333

# test connections :

```
rostopic pub /servos_absolute i2cpwm_board/ServoArray "servos:
- servo: 0
  value: 0.0" 
```

use appropriate servo and values to see actuation
