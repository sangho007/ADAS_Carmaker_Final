function y  = calculate_reference(pathx,pathy,egox,egoy,invalid) % path adn egox
    valid_indices = find(invalid > 0);
    if isempty(valid_indices)
        y = egoy;
        return
    end
    localPoints = zeros(length(valid_indices),2);
    localPoints(:,1) = pathx(valid_indices);
    localPoints(:,2) = pathy(valid_indices);
    


    polyFit = PolynomialFitting_class(length(localPoints)-1, length(localPoints),1e-1);
    
    % 다항식 fitting
    polyFit = polyFit.fit(localPoints);
    
    % 시각화를 위한 다항식 값 계산 클래스 인스턴스 생성
    polyVal = PolynomialValue_class(length(localPoints)-1, length(egox)); % 시각화를 위해 더 많은 점 사용
    
    % 계산된 다항식 값
    polyVal = polyVal.calculate(polyFit.coeff, egox);
    y = polyVal.y(1,1);
end