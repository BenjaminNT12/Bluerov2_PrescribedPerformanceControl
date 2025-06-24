function [qtin] = qtinVector(Edges, qt, leader,l, m)
    agent = 1;
    for k = 1:l
        for j = 1:2
            if Edges(k,j) == leader % leader
                % E(k,:)
                qtin(agent*m-2:agent*m,1) = qt(k*m-2:k*m,1);
                agent = agent + 1;
            end
        end
    end
    qtin(agent*m-2:agent*m,1) = zeros(m,1);
end