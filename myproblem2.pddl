(define (problem myproblem2) (:domain mydomain2)
(:objects 
robot0 - robot
valve0 valve1 - valve
pump0 pump1 - pump
wp0 wp1 wp2 wp3 wp4 wp5 wp6 wp7 wp8 wp9 wp10 wp11 - waypoint
charger0 charger1 charger2 - battery_station
r00 r01 r02 r03 r04 r05 r06 r07 r08 r09 - route
r10 r11 r12 r13 r14 r15 r16 r17 r18 r19 - route
r20 r21 r22 r23 r24 r25 r26 r27 - route
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

    (connects r00 wp0 wp7) ; charger wp0
    (connects r01 wp7 wp0)

    (= (distance wp0 wp7) 0.5) ; charger wp0
    (= (distance wp7 wp0) 0.5)

    (connects r02 wp1 wp7) ; valve0 wp1
    (connects r03 wp1 wp8)
    (connects r04 wp7 wp1)
    (connects r05 wp8 wp1)

    (= (distance wp1 wp7) 1)
    (= (distance wp1 wp8) 0.5)
    (= (distance wp7 wp1) 1)
    (= (distance wp8 wp1) 0.5)

    (connects r06 wp2 wp9) ; valve1 wp2
    (connects r07 wp2 wp10)
    (connects r08 wp2 wp11)
    (connects r09 wp9 wp2)
    (connects r10 wp10 wp2)
    (connects r11 wp11 wp2)
    
    (= (distance wp2 wp9) 0.5)
    (= (distance wp2 wp10) 0.5)
    (= (distance wp2 wp11) 1)
    (= (distance wp9 wp2) 0.5)
    (= (distance wp10 wp2) 0.5)
    (= (distance wp11 wp2) 1)


    (connects r12 wp3 wp5) ; charger wp3
    (connects r13 wp3 wp10)
    (connects r14 wp5 wp3)
    (connects r15 wp10 wp3)
    (= (distance wp3 wp5) 3)
    (= (distance wp3 wp10) 1)
    (= (distance wp5 wp3) 3)
    (= (distance wp10 wp3) 1)

    (connects r16 wp4 wp8) ; charger wp4
    (connects r17 wp4 wp11)
    (connects r18 wp8 wp4)
    (connects r19 wp11 wp4)
    (= (distance wp4 wp8) 2.5)
    (= (distance wp4 wp11) 1)
    (= (distance wp8 wp4) 2.5)
    (= (distance wp11 wp4) 1)
    
    (connects r20 wp5 wp7) ; pump0 wp5
    (connects r21 wp7 wp5)
    (= (distance wp5 wp7) 2.5)
    (= (distance wp7 wp5) 2.5)
    
    (connects r22 wp6 wp10) ; pump0 wp5
    (connects r23 wp6 wp11)
    (connects r24 wp10 wp6) ; pump0 wp5
    (connects r25 wp11 wp6)
    (= (distance wp6 wp10) 0.5)
    (= (distance wp6 wp11) 0.5)
    (= (distance wp10 wp6) 0.5)
    (= (distance wp11 wp6) 0.5)
    
    (connects r26 wp8 wp9) ; pump0 wp5
    (connects r27 wp9 wp8)
    (= (distance wp8 wp8) 0.5)
    (= (distance wp9 wp8) 0.5)
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
