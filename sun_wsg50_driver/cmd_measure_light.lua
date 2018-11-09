-- Mo-- Nicolas Alt, 2014-09-04
-- Command-and-measure script
-- Works with extended wsg_50 ROS package
-- Tests showed about 20Hz rate
--------------------------------------------
-- Mod 2017-03-30 (SUN )
-- Mod 2018-01-22 (Vanvitelli)
-- Tests - ~50Hz rate
--------------------------------------------

---CMD REGISTER-------------------------
cmd.unregister(0xB0);
cmd.unregister(0xB1);
cmd.unregister(0xB2);
cmd.register(0xB0); -- Measure only
cmd.register(0xB1); -- Position control
cmd.register(0xB2); -- Speed control
----------------------------------------
---VARS---------------------------------
FINGER_POSITION = 1; --CHANGE THIS TO SELECT FINGER
def_speed = 5;
is_speed = false;
B_SUCCESS = etob(E_SUCCESS);
--------------------------------------

---Fingers Init---------------------
N_SENSOR_BYTES = 50;
UART_BIT_R = 115200;
error_ = false;
function fingersInit()
    print("INIT\n")
    if finger.type(FINGER_POSITION) == "generic" then
        finger.power(FINGER_POSITION,true);
        finger.interface( FINGER_POSITION, "uart", UART_BIT_R );
        sleep(100);
        finger.write( FINGER_POSITION, "z");
        sleep(100);
        if finger.bytes_available(FINGER_POSITION) > 0 then
            rr = finger.read( FINGER_POSITION, 1 );
            if rr[1] == 120 then --120='x'
                printf("FINGER " .. FINGER_POSITION .. " OK\n");
            else
                printf("FINGER " .. FINGER_POSITION .. " ERROR no x!\n");
                error_ = true;
            end
        else
            printf("FINGER " .. FINGER_POSITION .. " no bytes avaiable!\n");
            error_ = true;
        end
     else
        printf("FINGER " .. FINGER_POSITION .. " ERROR NO GENERIC!\n");
        error_ = true;
     end
end

function process()
    id, payload = cmd.read();
    -- ==== Measurements (1) ====
    --busy = mc.busy()
    --blocked = mc.blocked()
    --pos = mc.position();
    
    if id == 0xB2 then
        -- do_speed = hasbit(payload[1], 0x02);
        cmd_speed = bton({payload[6],payload[7],payload[8],payload[9]});
        --print("set_speed");
        --is_speed = true;
        --def_speed = cmd_speed;
        mc.speed(cmd_speed);
    end
        
       
    -- ==== Actions ====
    -- Stop if in speed mode
    -- print(blocked, is_speed, pos);
    
    --if blocked and is_speed and pos <= 50 and cmd_speed < 0 then
     --   print("stop");
     --   mc.stop(); is_speed = false;
   -- end
    --if blocked and is_speed and pos >= 50 and cmd_speed > 0 then
   --     print("stop"); 
  --     mc.stop(); is_speed = false;
  --  end   
    
    finger.write( FINGER_POSITION, "a");
    finger_send = finger.read(FINGER_POSITION,N_SENSOR_BYTES);
        
    cmd.send(id, B_SUCCESS, finger_send);
       
end


--MAIN------------------------------
fingersInit();
while not error_ do   
    if cmd.online() then
        process()       
    end
end

printf("EXIT\n");