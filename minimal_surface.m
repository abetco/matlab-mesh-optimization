addpath('C:\Program Files\Mosek\8\toolbox\r2014a');
addpath(genpath('.\Examples\YALMIP-master'));
%
close all
N = 100;

%
[x,y] = meshgrid(linspace(0,1,N),linspace(0,1,N));
z = sdpvar(N,N,'full');

%
F = delaunay([x(:), y(:)]);
V = [x(:), y(:), z(:)];

% slack (for area)
t = sdpvar(size(F,1),1);

% % sanity check - optimize something stupid
% optimize(V(:,3)<=V(:,2), norm(V(:,3)-V(:,1)));

% find corner indices
F_x = x(F);
F_y = y(F);
E_len = sqrt(diff(F_x(:,[1 2 3 1]),1,2).^2 + diff(F_y(:,[1 2 3 1]),1,2).^2);
[~,E_len_max_ind] = max(E_len,[],2);
F_corner_ind = clockmod(E_len_max_ind-1,3);

% reshuffle F (so that corner vertex is always first)
J = clockmod([F_corner_ind, F_corner_ind+1, F_corner_ind+2],3);
I = ndgrid(1:size(F,1),1:size(F,2));
F = F(sub2ind(size(F),I,J));

% objective
h = sum(t);

% constraints:
% - area (for each triangle)
X=[(V(F(:,3),3)-V(F(:,1),3))' ; (V(F(:,2),3)-V(F(:,1),3))' ; ones(1,size(F,1))];
constraints=cone([t';X]);

frames=75;
clear mov;
mov(frames) = struct('cdata',[],'colormap',[]);
for j=1:frames
% boundary_z = x.^4.*y./2+log(y+0.1)-sqrt(x);
% boundary_z = sin((10+j)*x) + cos((7+j)*y*(x+1));
boundary_z = ones(size(x))*sin((j/3-1)/frames*8*pi)+0.25*sin(x*3*pi)+0.1*sin(y.*x*10*pi);
% boundary_z1 = ones(size(x))*sin((j-1)/frames*2*pi)+0.25*sin(x*9*pi)+0.1*sin(y.*x*11*pi);
boundary_z1=0.6*ones(size(x));
boundary_z = (1+j/50)*(x-0.5).^2 + (1+j/50)*(y-0.5).^2;
% boundary_z2 = ones(size(x))*sin((j-1)/frames*5*pi)+0.25*sin(x*9*pi)+0.1*sin(y.*x*11*pi);
% boundary_z = ones(size(x))*sin((j/5-1)/frames*8*pi)+0.25*sin(x*3*pi)+0.1*sin(y.*x*10*pi);
% figure(4);
% surf(x,y,boundary_z)

boundary_ind = find(x(:)==0 | x(:)==1 | y(:)==0 | y(:)==1);
bound = z(boundary_ind)==boundary_z(boundary_ind);
boundary_ind1 = find(x(:).^2+y(:).^2==1);
bound1 = z(boundary_ind1)==boundary_z1(boundary_ind1);
% boundary_ind1 = find((x(:)>0.25 & x(:)<0.26 & y(:)<=0.75 & y(:)>=0.25) | (x(:)>0.74 & x(:)<0.75 & y(:)<=0.75 & y(:)>=0.25)...
%                    | (y(:)>0.25 & y(:)<0.26 & x(:)<=0.75 & x(:)>=0.25) | (y(:)>0.74 & y(:)<0.75 & x(:)<=0.75 & x(:)>=0.25));
% bound1 = z(boundary_ind1) == boundary_z1(boundary_ind1);
% boundary_ind2 = find((x(:)>0.41 & x(:)<0.42 & y(:)<=0.6 & y(:)>=0.4) | (x(:)>0.61 & x(:)<0.62 & y(:)<=0.6 & y(:)>=0.4)...
%                    | (y(:)>0.41 & y(:)<0.42 & x(:)<=0.6 & x(:)>=0.4) | (y(:)>0.61 & y(:)<0.62 & x(:)<=0.6 & x(:)>=0.4));
% bound2 = z(boundary_ind2) == boundary_z2(boundary_ind2);
C= constraints + bound;
% + bound1;
% + bound2;

% riddle -- what does that do???
% if j>1
%     h = sum(t) + 0.8*norm(V-double(V),'fro');
% end

% optimize
optimize(C,h,sdpsettings('debug',true));


%
figure(1);
cla;
patch('faces',F,'vertices',double(V),'facecolor',[.6 0 .6],'edgecolor','none','facealpha',0.7);
axis vis3d;
cameratoolbar;
light;
% axis off;
axis manual;

if j==1
    pause
end
mov(j) = getframe;

% export
filename = sprintf('output/frame_%05d.vtk', j);
v_scalars.z = double(z);
t_scalars.area = 0.5*sqrt(sum(double(X).^2,1));
output_vtk_surf(filename,double(V),F,v_scalars,t_scalars,[]);
end


figure(2);
movie(mov);

% figure;
% z_hat = reshape(double(V(:,3)), size(x))
% surf(x,y,z_hat);

% read about movie+getframe
