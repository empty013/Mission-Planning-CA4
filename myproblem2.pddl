(define (problem myproblem2) (:domain mydomain2)
(:objects 
robot0 - robot
valve0 valve1 - valve
pump0 pump1 - pump
wp0 wp1 wp2 wp3 wp4 wp5 wp6 - waypoint
charger0 charger1 charger2 - battery_station
r00 r01 r02 r03 r04 r05 r06 r07 r08 r09 - route
r10 r11 r12 r13 r14 r15 r16 r17 r18 r19 - route
r20 r21 r22 r23 r24 r25 r26 r27 r28 r29 - route
r30 r31 r32 r33 r34 r35 - route
)

(:init
    ;todo: put the initial state's facts and numeric values here
    (= (robotSpeed) 1)
    ;(batteryCharged)
    ; (= (totalDistance) 0)
    ; (= (totalTime) 0)
    ; (= (battery_charge) 100)
    ; (= (battery_recharge_per_time) 3)
    ; (= (battery_drain_MOVE) 3)
    ; (= (battery_drain_INSPECT) 2)
    ; (= (battery_drain_PICTURE) 1)
    ; (= (duration_INSPECT) 10)
    ; (= (duration_PICTURE) 5)
    ; (= (duration_CHARGE) 7)
    (isAtWaypoint charger0 wp0)
    (isAtWaypoint valve0 wp1)
    (isAtWaypoint valve1 wp2)
    (isAtWaypoint charger2 wp3)
    (isAtWaypoint charger1 wp4)
    (isAtWaypoint pump0 wp5)
    (isAtWaypoint pump1 wp6)
    
    (robotLocation wp0)

    (connects r00 wp0 wp1) ; charger wp0
    (connects r01 wp0 wp2)
    (connects r02 wp0 wp5)
    (connects r03 wp0 wp6)
    (connects r04 wp1 wp0)
    (connects r05 wp2 wp0)
    (connects r06 wp5 wp0)
    (connects r07 wp6 wp0)
    (= (distance wp0 wp1) 3) ; charger wp0
    (= (distance wp0 wp2) 4)
    (= (distance wp0 wp5) 4)
    (= (distance wp0 wp6) 5)
    (= (distance wp1 wp0) 3)
    (= (distance wp2 wp0) 4)
    (= (distance wp5 wp0) 4)
    (= (distance wp6 wp0) 5)

    (connects r08 wp1 wp2) ; valve0 wp1
    (connects r09 wp1 wp3)
    (connects r10 wp1 wp4)
    (connects r11 wp1 wp5)
    (connects r12 wp1 wp6)
    (connects r13 wp2 wp1)
    (connects r14 wp3 wp1)
    (connects r15 wp4 wp1)
    (connects r16 wp5 wp1)
    (connects r17 wp6 wp1)
    (= (distance wp1 wp2) 1)
    (= (distance wp1 wp3) 3)
    (= (distance wp1 wp4) 4)
    (= (distance wp1 wp5) 3)
    (= (distance wp1 wp6) 3)
    (= (distance wp2 wp1) 1)
    (= (distance wp3 wp1) 3)
    (= (distance wp4 wp1) 4)
    (= (distance wp5 wp1) 3)
    (= (distance wp6 wp1) 3)

    (connects r18 wp2 wp3) ; valve1 wp2
    (connects r19 wp2 wp4)
    (connects r20 wp2 wp5)
    (connects r21 wp2 wp6)
    (connects r22 wp3 wp2)
    (connects r23 wp4 wp2)
    (connects r24 wp5 wp2)
    (connects r25 wp6 wp2)
    (= (distance wp2 wp3) 2)
    (= (distance wp2 wp4) 2.5)
    (= (distance wp2 wp5) 4)
    (= (distance wp2 wp6) 1.5)
    (= (distance wp3 wp2) 2)
    (= (distance wp4 wp2) 2.5)
    (= (distance wp5 wp2) 4)
    (= (distance wp6 wp2) 1.5)

    (connects r26 wp3 wp5) ; charger wp3
    (connects r27 wp3 wp6)
    (connects r28 wp5 wp3)
    (connects r29 wp6 wp3)
    (= (distance wp3 wp5) 2.5)
    (= (distance wp3 wp6) 1.5)
    (= (distance wp5 wp3) 2.5)
    (= (distance wp6 wp3) 1.5)

    (connects r30 wp4 wp5) ; charger wp4
    (connects r31 wp4 wp6)
    (connects r32 wp5 wp4)
    (connects r33 wp6 wp4)
    (= (distance wp4 wp5) 7)
    (= (distance wp4 wp6) 2.5)
    (= (distance wp5 wp4) 7)
    (= (distance wp6 wp4) 2.5)
    
    (connects r34 wp5 wp6) ; pump0 wp5
    (connects r35 wp6 wp5)
    (= (distance wp5 wp6) 4)
    (= (distance wp6 wp5) 4)
)

; (:goal (and
;     (robotLocation wp3)
;     (checked pump0)
;     (checked pump1)
;     (checked valve0)
;     (checked valve1)
; ))
(:goal (and 
    (robotLocation wp3)
    (checked valve0)
    (checked valve1)
    (checked pump0)
    (checked pump1)
    )
)

;un-comment the following line if metric is needed
(:metric minimize (total-Time))
)