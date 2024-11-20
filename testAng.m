            clc;

            % Definição das juntas
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
            q_Atual = q_PosIni


            [R, Td] = tr2rt(robot.fkine(q_PosIni));
            Rd = SO3(R);
            
            K = 60; % Define ganho

            %VelLim = 1;
            VelMed = 180;
            Count = 1;
            Direcao = [1 0 0];
            
            tam = 0.005;
            
            % inicia variáveis para o desenho
            J = robot.jacob0(q_PosIni); % Jacobiana geométrica
            Ti = robot.fkine(q_PosIni); % Cinemática direta para pegar a pose do efetuador 
            
            Ri = SO3(Ti); 
            
%             if comp.PCDropDown_2.Value
%                 
%                 Direcao = round(R.R*Direcao',2);
%                 Direcao = round(Direcao'*R.R,2);
%             else
% 
%                 Direcao = round(Direcao*R.R,2);
%             end

            while Count < 10
                Direcao = round(Direcao*Ri.R,2);
    
    
                if Direcao(1) ~= 0
                      ptraj = Ri*Ri.Rx(Direcao(1)*VelMed*tam*Count,"deg");
    
                elseif Direcao(2) ~= 0
                      ptraj = Ri*Ri.Ry(Direcao(2)*VelMed*tam*Count,"deg");
    
                elseif Direcao(3) ~= 0
                      ptraj = Ri*Ri.Rz(Direcao(3)*VelMed*tam*Count,"deg");
    
                end
    
                xrotD = Direcao(1)*VelMed*(pi/180);
                yrotD = Direcao(2)*VelMed*(pi/180);
                zrotD = Direcao(3)*VelMed*(pi/180);
    
    
                ptrajD = [xrotD; yrotD; zrotD];
                ptrajD = Ri.R*ptrajD;
                p_ant = transl(Ti);
                
                % Td.plot('rgb') % Plot pose desejada
                
                e_ant = 1; % Erro anterior
                e = 0; % Erro atual
                err_dif = e - e_ant;
    
                while (norm(err_dif) > 10e-3)
                    J = robot.jacob0(q_Atual); % Jacobiana geométrica
                    T = robot.fkine(q_Atual); % Cinemática direta para pegar a pose do efetuador 
                    p = transl(T); % Translação do efetuador
                
                    R = SO3(T); 
                    R = R.R(); % Extrai rotação do efetuador
                    p_err = p_ant-p; % Erro de translação
                    nphi = rotm2axang2(ptraj.R*R');
                    nphi_err = nphi(1:3)*nphi(4); % Erro de rotação (n*phi)
                    e_ant = e;
                    e = [p_err'; nphi_err']; % Vetor de erro
    
                    err_dif = e - e_ant;
    
                    u = pinv(J(:, 1:end))*(K*e + [0; 0; 0; ptrajD]); % Lei de controle
                    
                    q_Atual(1:end) = q_Atual(1:end) + 0.005*u'; % Cálculo de theta (Regra do trapézio)
                end
                
                %disp(q_Atual);
                robot.plot(q_Atual);
                Count = Count + 1;

            end