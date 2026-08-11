Len = size(KRcd,3);
K7Rcd = zeros(Len,7);
K8Rcd = zeros(Len,7);

for idx = 1:Len
K7Rcd(idx,:) = KRcd(7,1:7,idx);
K8Rcd(idx,:) = KRcd(8,1:7,idx);
end

%%
figure;plot(K8Rcd(:,1),'DisplayName','K8Rcd','Marker','*')