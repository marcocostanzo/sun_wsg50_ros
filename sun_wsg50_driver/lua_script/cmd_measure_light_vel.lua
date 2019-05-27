-- Mo-- Nicolas Alt, 2014-09-04
-- Modificato 2017-03-30 (SUN-MC)
-- Command-and-measure script
-- Works with extended wsg_50 ROS package
-- Tests showed about 20Hz rate

---CMD REGISTER-------------------------
cmd.unregister(0xB0);
cmd.unregister(0xB1);
cmd.unregister(0xB2);
cmd.register(0xB0); -- Measure only
cmd.register(0xB1); -- Position control
cmd.register(0xB2); -- Speed control
----------------------------------------
---VARS---------------------------------
def_speed = 5;
is_speed = false;
B_SUCCESS = etob(E_SUCCESS);
--------------------------------------

---Fingers Init---------------------
N_SENSOR_BYTES = 50;
UART_BIT_R = 115200;
function fingersInit()
    print("INIT\n")
    if finger.type(0) == "generic" then
        finger.power(0,true);
        finger.interface( 0, "uart", UART_BIT_R );
        sleep(100);
        finger.write( 0, "z");
        sleep(100);
        if finger.bytes_available(0) > 0 then
            rr = finger.read( 0, 1 );
            if rr[1] == 120 then --120='x'
                printf("FINGER 0 OK\n");
            else
                printf("FINGER 0 ERROR no x!\n");
            end
        else
            printf("FINGER 0 ERROR no bytes avaiable!\n");
        end
     else
        printf("FINGER 0 ERROR NON GENERIC!\n");
     end
end

finger0_send = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

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
    
    finger.write( 0, "a");
    finger0_send = finger.read(0,N_SENSOR_BYTES);
    
    speed = mc.speed();
        
    cmd.send(id, B_SUCCESS, ntob(speed), finger0_send);
       
end


--MAIN------------------------------
fingersInit();
while true do   
    if cmd.online() then
        process()       
    end
end