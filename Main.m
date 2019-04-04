function Main(serPort)

pi = 3.14159265359;

%function order
FindCenter();
LeaveRoom();
FindingBeacon();
TravelToBeacon();
[~, ~, incomeTh] = OverheadLocalizationCreate(serPort);
CircleBeacon();
BumpBeacon();
FindRoom();
WallFollow();
FindCenter();

    function FindCenter()
        %find the center
        disp('finding the center of the room');
        
        %turn horizontally
        [~, ~, th] = OverheadLocalizationCreate(serPort);
        ang = th * 180/pi;
        correction = 180 - ang;
        turnAngle(serPort, .2, correction);

        %if the gap infront is smaller than behind, turn around
        lidAll = LidarSensorCreate(serPort);
        Front = lidAll(340);
        Back = ReadSonar(serPort, 4);
        if (Back(1) > Front)
            turnAngle(serPort, .2, 180);
        end
        
        %align robot so that front and rear sensors are equal
        lidAll = LidarSensorCreate(serPort);
        Front = lidAll(340);
        Back = ReadSonar(serPort, 4);
        while(abs(Front-Back(1))>0.05)
            lidAll = LidarSensorCreate(serPort);
            Front = lidAll(340);
            Back = ReadSonar(serPort, 4);
            
            %slow down as approaching the center
            speed = abs(Front-Back(1));
            if (speed > 0.5)
                speed = 0.5;
            end
            
            SetDriveWheelsCreate(serPort, speed, speed);
        end

        
        %turn 90 degrees
        turnAngle(serPort, .2, 90);
      
        %if the gap infront is smaller than behind, turn around
        lidAll = LidarSensorCreate(serPort);
        Front = lidAll(340);
        Back = ReadSonar(serPort, 4);
        if (Back(1) > Front)
            turnAngle(serPort, .2, 180);
        end
        
        %align robot so that front and rear sensors are equal
        lidAll = LidarSensorCreate(serPort);
        Front = lidAll(340);
        Back = ReadSonar(serPort, 4);
        while(abs(Front-Back(1))>0.25)
            lidAll = LidarSensorCreate(serPort);
            Front = lidAll(340);
            Back = ReadSonar(serPort, 4);
            
            %slow down as approaching the center
            speed = abs(Front-Back(1));
            if (speed > 0.5)
                speed = 0.5;
            end
            
            SetDriveWheelsCreate(serPort, speed, speed);
        end
    end




    function LeaveRoom()
        %point towards the door
        disp('leaving the room');
        [~, ~, th] = OverheadLocalizationCreate(serPort);
        
        ang = th * 180/pi;
        correction = 180 - ang;
        turnAngle(serPort, .2, correction);
        
        lidAll = LidarSensorCreate(serPort);
        lidBL = lidAll(440);
        lidBR = lidAll(210);
        pause on;
        
        %loop whilst two forwardish facing lidars are too far away
        while(lidBL > 0.5 && lidBR > 0.5)
            
            lidAll = LidarSensorCreate(serPort);
            
            %get the heading of the robot in radians
            [~, ~, th] = OverheadLocalizationCreate(serPort);
            %flip the heading
            th = th + pi;
            if(th > pi)
                th = th - 2*pi;
            end
            %change the heading to degrees
            ang = th * 180/pi;
            lid = round(ang * (681/240));
            
            %find the top and bottom lidar
            lidL = lidAll(595 - lid);
            lidR = lidAll(85 - lid);
            
            %find the difference between the lidar
            lidDiff = lidR - lidL;
            
            %if the difference is significant, turn away from the closer
            %side
            if (abs(lidDiff) > 0.1)
                [~, ~, th] = OverheadLocalizationCreate(serPort);
                ang = th * 180/pi;
                correction = 180 - ang - (lidDiff*25);
                
                turnAngle(serPort, .2, mod(correction, 360));
            end
            
            %drive forward
            SetDriveWheelsCreate(serPort, 0.5, 0.5);
            pause(0.1);
            
            %update the forwardish facing lidars
            lidBL = lidAll(440);
            lidBR = lidAll(210);
        end
        
        %alignment has been found, exit the room
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        pause(2);
        SetDriveWheelsCreate(serPort, 0, 0);
        
        %turn left
        [~, ~, th] = OverheadLocalizationCreate(serPort);
        ang = th * 180/pi;
        correction = 90 - ang;
        turnAngle(serPort, .2, correction);
        
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        pause(2);
        SetDriveWheelsCreate(serPort, 0, 0);
    end



    function FindingBeacon()
        disp('finding the beacon');
        %wall follow until beacon is found
        %Init
        followDist = 0.5;
        cycle = 0;
        pGain = 50;
        iGain = 5;
        dGain = 10;
        iErrorSize = 10;
        iError = zeros(1,iErrorSize);
        dPrevError = 0;
        
        
        %whilst beacon is not found, wall follow
        found = false;
        while (found == false)
            %PID controller to follow right wall
            SetDriveWheelsCreate(serPort, 0.5, 0.5);
            pause(0.1);
            
            lidAll = LidarSensorCreate(serPort);
            [lidarM,] = min(lidAll(1:340));
            error = followDist - lidarM;
            
            %Proportional
            pOut = pGain * error;
            
            %Integral
            iError(mod(cycle,iErrorSize) + 1) = error;
            iOut = iGain * sum(iError);
            
            %Derivative
            dOut = dGain * (error - dPrevError);
            dPrevError = error;
            
            %Course Correction
            correction = pOut + iOut + dOut;
            turnAngle(serPort, .2, correction);
            
            
            %check camera
            camera = CameraSensorCreate(serPort);
            
            %if a beacon is spotted
            if(any(camera))
                ang = camera * 180/pi;
                ang = ang + 120;
                lidAng = ang * 681/240;
                
                %and the beacon is within range
                if(lidAll(round(lidAng)) < 4)
                    %the beacon has been found
                    found = true;
                end
            end
        end
    end



    function TravelToBeacon()
        disp('heading to beacon');
        %travel to beacon
        lidAll = LidarSensorCreate(serPort);
        lidF = lidAll(340);
        
        %while the beacon is too far away
        while(lidF > 0.75)
            %turn towards the beacon
            camera = CameraSensorCreate(serPort);
            lidAll = LidarSensorCreate(serPort);
            lidF = lidAll(340);
            turnAngle(serPort, .2, camera*10);
            
            %drive forward
            SetDriveWheelsCreate(serPort, 0.5, 0.5);
            pause(0.5);
        end
    end



    function CircleBeacon()
        disp('circling the beacon');
        %circling the beacon
        
        followDist = 0.5;
        cycle = 0;
        pGain = 100;
        iGain = 5;
        dGain = 5;
        iErrorSize = 10;
        iError = zeros(1,iErrorSize);
        dPrevError = 0;
        
        %while the wall has not been looped
        halfLooped = false;
        looped = false;
        while (looped == false)
            %PID controller to follow the wall on the right
            SetDriveWheelsCreate(serPort, 0.5, 0.5);
            pause(0.1);
            
            lidAll = LidarSensorCreate(serPort);
            [lidarM,] = min(lidAll(1:200));
            error = followDist - lidarM;
            
            %Proportional
            pOut = pGain * error;
            
            %Integral
            iError(mod(cycle,iErrorSize) + 1) = error;
            iOut = iGain * sum(iError);
            
            %Derivative
            dOut = dGain * (error - dPrevError);
            dPrevError = error;
            
            %Edge Case
            eOut = 0;
            if (lidAll(85) > followDist && lidAll(1) > followDist)
                if(lidAll(340) > followDist * 2)
                    pOut = 0;
                    iOut = 0;
                    dOut = 0;
                end
                eOut = -10;
            end
            
            %Course Correction
            correction = pOut + iOut + dOut + eOut;
            turnAngle(serPort, .2, correction);
            
            
            %check to see if the robot has looped the wall
            if(~halfLooped)
                %if the robot is facing down, it has completed half a loop
                [~, ~, th] = OverheadLocalizationCreate(serPort);
                if(-th > (pi/2)-0.5 && -th < (pi/2)+0.5)
                    halfLooped = true;
                end
            else
                %if the robot has completed half a loop, and is facing up,
                %then it has completed a full loop
                [~, ~, th] = OverheadLocalizationCreate(serPort);
                if(th > (pi/2)-0.5 && th < (pi/2)+0.5)
                    looped = true;
                end
            end
        end
        
        SetDriveWheelsCreate(serPort, 0, 0);
    end



    function BumpBeacon()
        disp('bumping beacon');
        %Bumping the beacon
        
        %align upwards
        [~, ~, th] = OverheadLocalizationCreate(serPort);
        ang = th * 180/pi;
        correction = 90 - ang;
        turnAngle(serPort, .2, correction);
        camera = CameraSensorCreate(serPort);
        
        %whilst the beacon is still in sight (still ahead of the robot)
        while (any(camera))
            %drive forwards
            camera = CameraSensorCreate(serPort);
            SetDriveWheelsCreate(serPort, 0.5, 0.5);
        end
        pause(0.2);
        SetDriveWheelsCreate(serPort, 0, 0);
        
        %turn towards the beacon
        turnAngle(serPort, .2, -90);
        
        %back away from the beacon
        SetDriveWheelsCreate(serPort, -0.5, -0.5);
        pause(0.2);
        
        %bump the beacon
        lidAll = LidarSensorCreate(serPort);
        lidF = lidAll(340);
        while(lidF > 0.05)
            lidAll = LidarSensorCreate(serPort);
            lidF = lidAll(340);
            SetDriveWheelsCreate(serPort, 0.5, 0.5);
        end
        
        %back away from the beacon
        SetDriveWheelsCreate(serPort, -0.5, -0.5);
        pause(0.2);
        SetDriveWheelsCreate(serPort, 0, 0);
    end



    function FindRoom()
        disp('refinding the room');
        %Refinding the room
        
        %turn away from the angle the robot approached the beacon from
        %if the beacon was approached from the left, turn towards the left
        outgoAng = incomeTh * 180/pi - 180;
        [~, ~, th] = OverheadLocalizationCreate(serPort);
        ang = th * 180/pi;
        correction = outgoAng - ang;
        turnAngle(serPort, .2, correction);
        
        %drive forwards until the room is 0.5m away
        lidAll = LidarSensorCreate(serPort);
        [lidF,] = min(lidAll(210:440));
        while(lidF > 0.5)
            SetDriveWheelsCreate(serPort, 0.5, 0.5);
            lidAll = LidarSensorCreate(serPort);
            [lidF,] = min(lidAll(210:440));
        end
        
        SetDriveWheelsCreate(serPort, 0, 0);
    end



    function WallFollow()
        followDist = 0.35;
        cycle = 0;
        pGain = 50;
        iGain = 5;
        dGain = 10;
        iErrorSize = 10;
        iError = zeros(1,iErrorSize);
        dPrevError = 0;
        
        found = false;
        while (~found)
            %PID controller for wall on the left
            SetDriveWheelsCreate(serPort, 0.5, 0.5);
            pause(0.1);
            
            cycle = cycle + 1;
            
            lidarRes = LidarSensorCreate(serPort);
            [lidarM,] = min(lidarRes(509:681));
            
            error = followDist - lidarM;
            
            %Proportional
            pOut = pGain * error;
            
            %Integral
            iError(mod(cycle,iErrorSize) + 1) = error;
            iOut = iGain * sum(iError);
            
            %Derivative
            dOut = dGain * (error - dPrevError);
            dPrevError = error;
            
            %Course Correction
            correction = pOut + iOut + dOut;
            
            if(correction > 25)
                correction = 25;
            end
            if(correction < -25)
                correction = -25;
            end
            
            turnAngle(serPort, .2, -correction);
            
            %found is true if the robot is heading in to the room
            [~, ~, th] = OverheadLocalizationCreate(serPort);
            if(th > -0.5 && th < +0.5)
                found = true;
            end
        end
        
        %drive in to the room
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        pause(2);
        %turn
        [~, ~, th] = OverheadLocalizationCreate(serPort);
        ang = th * 180/pi;
        correction = 90 - ang;
        turnAngle(serPort, .2, correction);
        %drive some more
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        pause(2);
        
    end

end