            L(1) = Revolute ('alpha', pi/2, 'a', 0.15, 'd', -0.45);
            L(2) = Revolute ('alpha', pi, 'a', 0.59, 'offset', -pi/2);
            L(3) = Revolute ('alpha', -pi/2, 'a', 0.13, 'offset', pi/2);
            L(4) = Revolute ('alpha', -pi/2, 'a', 0, 'd', -0.6471);
            L(5) = Revolute ('alpha', pi/2);
            L(6) = Revolute ('alpha', pi, 'd', -0.095, 'offset', pi);
            % Concatenação das juntas
            robot = SerialLink(L, 'name', 'COMAU Smart Six');
            
            Tbase = SE3([1 0 0 0 ; 0 -1 0 0 ; 0 0 -1 0 ; 0 0 0 1]);
            robot.base = Tbase;
            Ttool = SE3([1 0 0 0; 0 1 0 0; 0 0 1 0.15; 0 0 0 1]);
            robot.tool = Ttool;
            
            q_PosIni = [0 0 -pi/2 0 pi/2 0];
                
            robot.plot(q_PosIni);