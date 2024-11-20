function CloseArm()
    ADDR_TORQUE_ENABLE          = 64;
    ADDR_GOAL_POSITION          = 116;
    ADDR_PRESENT_POSITION       = 132;
    DXL_MINIMUM_POSITION_VALUE  = 0; % Dynamixel will rotate between this value
    DXL_MAXIMUM_POSITION_VALUE  = 4095; % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    BAUDRATE                    = 57600;

    PROTOCOL_VERSION            = 2.0;          
    
    % Factory default ID of all DYNAMIXEL is 1
    DXL_ID                      = 1; 
    
    % Use the actual port assigned to the U2D2. 
    % ex) Windows: 'COM*', Linux: '/dev/ttyUSB*', Mac: '/dev/tty.usbserial-*' 
    DEVICENAME                  = 'COM4';       
    
    % Common Control Table Address and Data 
    ADDR_OPERATING_MODE         = 11;          
    OPERATING_MODE              = 3;            % value for operating mode for position control                                
    TORQUE_ENABLE               = 1;            % Value for enabling the torque
    TORQUE_DISABLE              = 0;            % Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold
    
    ESC_CHARACTER               = 'e';          % Key for escaping loop
    
    COMM_SUCCESS                = 0;            % Communication Success result value
    COMM_TX_FAIL                = -1001;        % Communication Tx Failed
    
    port_num = portHandler(DEVICENAME);

    while 1
        if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
            break;
        end
    
        % Write goal position
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, typecast(int32(dxl_goal_position(1)), 'uint32'));
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end
    
        while 1
            % Read present position
            dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
            dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
            if dxl_comm_result ~= COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
            end
    
            fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', DXL_ID, dxl_goal_position(index), typecast(uint32(dxl_present_position), 'int32'));
    
            if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
                break;
            end
        end
    
    end

end