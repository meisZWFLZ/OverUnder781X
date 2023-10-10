# Iterative Turn PID turning
Starts at: 
  - kP=2
  - kD=10
  - smallErr=1 deg
  - smErrTimeout=100 ms
  - largeErr=3 deg
  - lgErrTimeout=500 ms
  - slew = 0

### Tests:
1. no change
  - 118 deg
2. smErrTime = 500ms
  - 119 deg
3. lgErrTime = 5000ms
  - 119 deg
4. smErrTime = 500ms
  - 117 deg
5. kP = 4
  - 124 deg
6. Since it also moves when turning, maybe it is turning toward that point
- turnTO(30,0) -> turnTo(1000, 0)
- 93 deg
7. reset PID values
- 88.7 deg
8. smErrTime = 250
- 87.8 deg
9. Replace battery
- 88.1 deg
10. kP = 5
- 92.4 deg
- Oscillates
11. kD=20
- 92.4 deg
- Oscillate less
12. kD-30
- 93.01
- Barely any Oscillation
13. kP=10
- 82.3
- Big oscillation
14. kD=50
- 91.6
- less oscillation
15. kD=75
- 91.3
- Quick small oscillation
16. kD=85
- 89.7
- Smaller oscillation
17. kD=90
- 89.4
- even smaller oscillations
18. kP=9.5
- 89.1
- tinier oscillations
19. smErrTimeout =100ms
- 89.7
- small oscillations
20. kP=9
- 88.7
- small oscillations
21. kD=105
- 89.4
- small oscillations
22. kD=300
- very oscillaty
23. kD=0
- inaccurate
- Crazy oscillations
24. kD=200
- oscillates a lot around target heading
25. kD=100
- less small oscillations
26. kD=80
- one small oscillation
27. kD=70
- no oscillations
- 91.5