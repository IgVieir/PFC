function DisconnectServo()
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


    % Disable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    
    % Close port
    closePort(port_num);
    
    % Unload Library
    unloadlibrary(lib_name);
    
    close all;
    clear all;

end
