function plotInitialFormationShape(agentPositions, edgeArray)
    figure;
    view([-45, -90, 60]);
    grid on;
    Framework3Dplot(agentPositions, edgeArray);
end
