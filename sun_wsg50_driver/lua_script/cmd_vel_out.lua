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
def_speed = 5;
is_speed = false;
B_SUCCESS = etob(E_SUCCESS);
--------------------------------------

function process()
    id, payload = cmd.read();
    -- ==== Measurements (1) ====
    --busy = mc.busy()
    --blocked = mc.blocked()
    pos = mc.position();
    speed = mc.speed();
    
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
        
    cmd.send(id, B_SUCCESS, ntob(pos), ntob(speed));
       
end


--MAIN------------------------------

printf("====OLD VEL PID GAINS====\n");
kp,ki,kd=mc.pid();
print("KP");
print(kp);
print("KI");
print(ki);
print("KD");
print(kd);
printf("-----------------------\n");

mc.pid( 0.1, ki, kd );

printf("====NEW VEL PID GAINS====\n");
kp,ki,kd=mc.pid();
print("KP");
print(kp);
print("KI");
print(ki);
print("KD");
print(kd);
printf("-----------------------\n");

while not error_ do   
    if cmd.online() then
        process()       
    end
end

printf("EXIT\n");