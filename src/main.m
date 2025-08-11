clear; close all; clc

% Fixing folders, relative to file (src)
thisFile  = mfilename('fullpath');   
srcDir    = fileparts(thisFile);     
rootDir   = fileparts(srcDir);       
plotsDir   = fullfile(rootDir,'plots');
resultsDir = fullfile(rootDir,'results');

if ~exist(plotsDir,'dir'),   mkdir(plotsDir);   end
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

% Constants (feel free to adjust, these ones are from Cessna 172)

Clift = 1.05; 
Cdrag = 0.048;
g = 9.81;
m = 1040;
rho = 1.225;
A = 16.2;

% Some Example airfoils (from FAA data sheet).

DemoAirfoils = {'FlatPlate', [0.4, 0.08]; 'Eppler387',  [0.8 0.021]; 'NACA2412',   [0.9 0.018]};



% Cessna 172, flaps up + best glide: 1040m, 16.2m^2, Cl=1.05, Cd = 0.048

params = [Clift, Cdrag, g, m, rho, A]; % Physical paramaters vector

% Initial conditions (position, velocity)


h0 = 100; % Release height
x0 = 0; 
Vbg = sqrt( 2*m*g / (rho*A*Clift) );   % This ensures best glide speed, lift = weight
vx0 = Vbg;                              
vy0 = 0; 

y0 = [vx0, vy0, x0, h0]; % Initial conditions vector


tspan = [0, 600]; 

options = odeset('Events', @GroundHit);
%[t, y] = ode45(@(t,y) gliderODE(t,y, params), tspan, y0, options);
%range = y(end, 3); % Gives x distance when y = 0

%Plotting + returning range/flight times

%PlotResults(y); 



%fprintf('Range = %.1f m, Flight Time = %.1f s\n', Range, Time);


% Create result arrays + plot prep
n = size(DemoAirfoils,1);
ranges = zeros(n,1);
times  = zeros(n,1);

figure; hold on %[output:774da621]

%Looping over airfoils array, plotting trajectory:

for k = 1:n
    CLk = DemoAirfoils{k,2}(1);
    CDk = DemoAirfoils{k,2}(2);

    params(1) = CLk;     % Overriding Cl, Cd as defined earlier
    params(2) = CDk;    

    Clift = params(1);
    Vbg   = sqrt( 2*m*g / (rho*A*Clift) );
    y0    = [Vbg, 0, x0, h0];

    [t, y] = ode45(@(t,y) gliderODE(t,y,params), tspan, y0, options); % Calling ode solver

    plot(y(:,3), y(:,4), 'LineWidth', 1.5);   % Plotting h vs x %[output:774da621]
    ranges(k) = y(end,3);
    times(k)  = t(end);
end

xlabel('Distance (m)'); ylabel('Height (m)'); % Graph detials (labels, title, legend) %[output:774da621]
title('Height vs. Distance (Sample Airfoils)'); %[output:774da621]
legend(DemoAirfoils(:,1), 'Location','best'); grid on; hold off %[output:774da621]


% Saving trajectories
exportgraphics(gcf, fullfile(plotsDir,'trajectories.png'), 'Resolution',200); %[output:774da621]


% Print a table of results: 
fprintf('\n%12s   %6s  %6s   %9s   %8s\n','Airfoil','CL','CD','Range (m)','Time (s)'); %[output:936cce49]
for k = 1:n %[output:group:23c64573]
    fprintf('%12s   %6.2f  %6.3f   %9.1f   %8.1f\n', ... %[output:31d42e7b]
        DemoAirfoils{k,1}, DemoAirfoils{k,2}(1), DemoAirfoils{k,2}(2), ranges(k), times(k)); %[output:31d42e7b]
end %[output:group:23c64573]

% Writing results table
T = table( string(DemoAirfoils(:,1)), ...
           cellfun(@(v)v(1), DemoAirfoils(:,2)), ...
           cellfun(@(v)v(2), DemoAirfoils(:,2)), ...
           ranges, times, ...
           'VariableNames', {'Airfoil','CL','CD','Range_m','Time_s'});
writetable(T, fullfile(resultsDir,'summary.csv'));

% Here we compare the different airfoils:

figure %[output:0625a033]
bar(ranges); grid on %[output:0625a033]
set(gca,'XTickLabel', DemoAirfoils(:,1)); %[output:0625a033]
ylabel('Range (m)'); title('Glide Range vs. Airfoil'); %[output:0625a033]

% Saving bar chart
exportgraphics(gcf, fullfile(plotsDir,'range_by_airfoil.png'), 'Resolution',200); %[output:0625a033]


% Best airfoil summary
[bestRange, idx] = max(ranges);
fprintf('\nBest: %s  ,  Range = %.1f m  ,  Time = %.1f s\n', ... %[output:group:1769a35c] %[output:2f69d925]
        DemoAirfoils{idx,1}, bestRange, times(idx)); %[output:group:1769a35c] %[output:2f69d925]





%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":10.7}
%---
%[output:774da621]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAOsAAACwCAYAAADuQ5nhAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ2QV8WRb2AVXBZclO8FkzXc+UEkckpE4xm8goveJSbBULocQoQ7UeACCWBODBUSDCbgBxhXInUg6hZruTmqTvQODytyRE4IpjQYol4wKB9LkK\/FFRZwZe961v7T\/9l57837mDfvsfOqKHb3zZuZ7p7fdE93z0yHlpaWFnCP44DjQOY50MGBNfMych10HBAccGB1A8FxICcccGDNiaBcNx0HlGD9xS9+AQsXLoS7774b7rzzzgKXXnrpJbjjjjtg9OjR8MADDwRy780334Rx48bB4MGDYdmyZVBWVhb4zaxZs2D16tWi\/MiRI5XlP\/roI5g5cyZMmzYNLrvsssA6ky5A\/JHrHT58eBGdH3zwAYwZM0YUq6urg969ewd2Bb+ZMWMGLF68WKt8YIWGCqCctmzZUkQXybuxsVG0KvPDUFd8q40yBr0qpLE5cODANvLUlTWOXcTQpk2bRDMyxlRtU91GwRpFOEFgJWK3bdsGNTU1mQIr0qsSpC4fdAWuW5+pcgSAu+66qzCZU9937dpV1KxtwCYFVpk+HZCp+E\/9GTVqlJbC43XEBqs8m3IiVIzi5VFDDxo0SGhx0qQE1oceegieffZZMQMRAEpLS4tmJdXMRGDes2dPYfZTAVzWjrrWArapsjz4jEl1qcBH1gkJgQbzsWPHhBbmgx15gu\/5TCzTTH257bbbYP369eL7bt26tZnIOL2q98R3nQkHy65bt66oDdnqItobGhoK5WStwmnhMho\/fjxUV1cXNA9aZ8QDPhly\/o4YMQKefvpp8Q2XZdAY1NVuxD8EGY5J2VqUZY31klVVUVEhvkFZkkbF91wO8rjgOEpEs8pApQFIwJMZpRqQl156KfzhD39oA1Z5VkIBzJs3LxCsHExe\/UCGofDlR3e29FomeNGL7aAZvG\/fPrEsIDOR2kfasG0VWNeuXSuWBfzhQtYxyVVleB0cqPIkIi9dvLQ\/HwteSxhVO9SPysrKNrKlAX3uuefC7t27CyyQJ0NZm3MAyjLZsWOHUgZByy4cL2jNPf744\/DII4+In7ll5wVW3repU6cWJiEOVhwXqvEo0+mrWb3MKKqEBgERSh3GmQT\/RoyhWQiJQy2q0iayZqU2ZGZjn4hxXmawbGrI4KJZTBecMh+8wOo3u3KweplAOmawvEygvhBPiefl5eVigqAZnms43n\/kM04SJDMEpyxXTr+XWanSmkFLAk4LtyBILvSeaKNJ1m\/8kGxlfshj0GvMqvwqMs00jlXajyZm4juClU8EsgXC+Sb3iWTWp08fIaNYYFXNlNyMIk1CjELNKDuP5IEhD0Z5AtABKzeFV65cCffeey9ws1i1vvKbWZMCa5AZ7wdWWTtSf+WJQ14GyDKQB6OXdeRlHgY5GeUxEWSSYzvc3OcaS6bNy3LhE00Q\/TQGZZn6TSxyP+QxiTwNmqjJuSjzz2vy47gYMmRIMFiDvMFeYCUBIUO4NzgtsHJTGE2Pp556ClTaTB6ouuvWqGYwCUyeLGRNQbMzlpcnLy\/NSrIKGqxhwKriRxBYCQScRllTylZUXLByfgXR7wVW1aSC9aosBqKRfxMVrCrg8\/GLvEkErH7mEjboZT7omMFeZoqOZuVt0\/rQT3PqmJ98JvYCKwGJBmdQvbLji8wd2ZSi37lm1tWsKnOMA47Wylw7eS1\/8O9eYFXxRGVue9FCZnAUzcpNzbBmsB+tnF6vcl7OROI7n0hU\/EvMDA7SrF4mlNd6M4yDKQis5FXz04ZecTEvp4xXv2Vt5PU9XwKgVpTBunXrVqUjwcsjPGXKFFizZk2Rh5gGjS5YsR9RHExemsbLbPMaC1QPTUQqZ1BczZqEg8lrHMkTMPGf6JV9A14TrZcZ7DchhHIwBYFVpcF4XC3IbY7146MK3fg5ALib2w+sfiabPICD+q3SrPJsq5MUIbvo5bUS7xfyB0Nb5CnE+r\/zne\/A5MmTC2Z90JqVBonXmpdokJc0XpaIKjQmD2CyZmTAc9plWsjTH0WzYvtxQjdesWBV+IlolTUimapRwKoCrHboJsg0iPqehMUHaFASRNS23HfmOIDAX7p0qbWkFKIsaJlhjgN2ajaWG0wzz8SJEwtpg6hp0QV98uTJImoJvDwOaTvzxY448tGqKoPJRs8dWBPgumrRTCYzgvfgwYOFVtBpUltbCxQUv+6664QHGU0\/\/JnnJifQNVdFQhxAi6i+vl475zuhZouqcWCNyVW+TsXY5ty5cwuaFc2nDRs2FASMAsc1GQISv6MMD0zOR5N5xYoVVgdDTFa4zx0HEuVA4mYwZtDg07VrV2HycrAiOPGhHTv8dwTn\/PnzC\/m88u9ENaad9e3bN1EmuMocB5LiQElJSVJVtakncbDKi38ZrKRJsRxq2u3btwvwypoUNe2cOXNg+fLlha1iCNRnHp8Nx06Vw4Tx40VTZx1+EVrO7gsne3wFOp78M5w6uy2QC38\/vBugxwDxXVDZ3+45DldUdDnNsHc3QUnfLtDc9fLEhdHc3AwY1sIlgUlhJ97xgArbI10YyunRo4cRVucKrBhbLXu7CvqfZ4QXoSrFSeHUWX2hY2kFdCptnQBKzh\/eOoH0bP1f92lqahJJ\/gMGDDijwNoe6cLJ1tSEmzpY45jBCNaxY8eKdS8ObJ1n56HjhWIDuhwA+n3n4da\/09+a978Hu080g\/j\/ZDPsOX4+fKPsv6FjWQkMK9km\/g\/zdPwUwJ0H3iw+O+eiGZ6fHz9+HPbu3Su2ApoSdJi+J1XW0ZUUJ1vrSRWs3OzFxmUHEzd7VQ6mKGCNyi4E9cZ3G2DnoSZ4ZXsD\/OmdtwWw+3+yDwae3wAVn\/48rOT3AsglfZjJ7NEoAvjUhz3hnMFfLgKvG9RRpWTnO1vyShWs3OOLbObeXwr3+IVu0gSrahgggFtBfFi8RhAjoPs1\/1n8fsWJ3wlAt2rj34vfz\/5cmSeQEbyoeXFtd6D0ZqdZ7WAvdKvtAqzIFTmdjsdReX6pKinCNli9AFy7ZW8BuHKZL32uHIafewRm\/XUJnDr+Gnx8YDM0H2w9f0d+CLxBZnPo0WXpA1uD2jS5tugypllNMCyLYOV0otb1A+73v\/JZ+P5XKsW6+OiWGujY\/SA0vfWMr+btMmgydOjUBToP\/JYJlhqt09agNkoUANiiy4HVkGT9gEugxaYRuIfX\/SscPbQdzj78oggPqda\/pHX9HFWGSIlcra1BHbnDmh\/aosuBVVNAcYoRcH\/24nuFai44rwtUDesrNC0Jv1\/nT6DplRpo3v8+HH2tBs6+sEysebknmkCLmpY8znH6ZvJbW4PaJE1Yty26HFhNS5bV7wXaMUN7wthLS4ocTKhxG19eWQTcLl8oL+otxnU7X3BzZk1kW4PatEht0eXAalqyivpVoO3fvQRuu3qA0LTyQ8A98f5G4aSSQZtVE9nWoDYtUlt0ObCalqxP\/V6a9tFbL4FrBxVrUaqGgNtS8jsBXJWJnJV1ra1BbVqktuhyYDUtWY36\/7e+AVZs2AHLftNQKM2dUF5VkFf540P\/Dh27Hygqhmvacy6abnVda2tQa7A8VhFbdDmwxhJbMh+T8Fu69oLpdX8UiRb4oBPquSlDxf9Bz5H\/uA+O\/vbBTJnItgZ1EK\/ivrdFlwNrXMkl8L0s\/J+9uAPIc4xA9TOLefNkIje9s0TpRe56+aLQmwzikGdrUMfps863tuhyYNWRjuEyKuFjKuNNj70eyizm69oPqm+Hk++\/Al2GlAvg0oOmcVprWluD2rC4XOhGh8FZz2DSoUFVxmtQowNqau1bkcxibOfws\/PgcN2PRJJF6TU9C84o9B6noWUdWKOOCPV3TrMmy89ItQUN6qhmMXaGTGMELWpZHvZBJ1TXoYsi9VnnoyC6dOrIYhlbdDmwZmA06Ag\/jllMoCXTGLUspTSa1LI6dGWA\/aG7YIsuB9bQokr+A13hy2Yx7uhZM3VoqA6RaSxrWcyG6v6l2lB1BRXWpSuonqy9t0WXA2sGRkJY4eM6tnZL6x7aKIAl0\/jIf\/5ErGW5lsX9tUk5oMLSlQFRaHXBFl0OrFriMVsoivDjApabxtDpjaK1LGrZsqGLYidURKHLLKeTqd0WXQ6sycgvVi1Rhc8Bizt4qqsuidQPNI1lLYsVxQ3zRKUrEhEpfmSLLgfWFIXs1VQc4X+t+vVCaCcOYMk0btzwMyj729PHucZxQMWhKwNi8eyCLbocWDMwKuIKPynAkml8YOU\/tdndgyGesKdVxKUrA6JRdsEWXQ6sGRgRSQifA1ZnE0AQ2Y3rV8LhX95VlEwR1mOcBF1B\/bTx3hZdDqw2pC21mZTwOWBx\/YpmcZynadt62DvvegFYSlkMA9ik6IpDg4lvbdHlwGpCmiHrTFL4pgDL47K669gk6QrJUqPFbdHlwGpUrHqVJy18U4DFeCw5n3QAmzRdetw0X8oWXQ6s5mUb2IIJ4XPA4p5Yr5MnAjv3aQH0Fu+cUikSKLi3uNs1tZ7b7kzQpdtfk+Vs0dUGrPwiZJlg27eRt7ddN3EHnAnAUn5x92+evmvIy1Nsa1DH5VvQ97boKoCVTsPHjtbU1ABeaCw\/eP8M3kjerVs3zzJBhMZ578AannsmActTFVUJFLYGdXguhfvCFl0CrHjd+8KFC2HevHlQVnZ6o7IXCah9sSxdihyO1OilHVjD8w6T\/3ETO\/4fJY\/Yq8X6edfD8W3rhUlMucWyp9jWoA7PpXBf2KLLrVnDyclIadPC59vr4mQ5ycQHAdY0XUaEoVGpLbocWDWEY7pIGsLnG9iTcDgRT\/ZX3w6YQCGHdspH\/tra8SdngrxUNLQBK61LVYXxst+6ujro3bu3aX4o63dmcDy28\/XrGz+4WuvURJ0WaY8sJk7gOhYfDO10ung+HPj4M+4qSx0mapQpAiuuXceMGQNVVVXAr2LUqCeVIg6s8diM69bL73tVVJLk+hXrQ+2KWlYO7Ry98CHod\/HX3I3u8UQnvm4D1kmTJsGCBQuU3uAE2otVhQNrLPaJj\/n6NYkcYt4jSk8ME4uNT1H6NaSxbNEyg\/Gy4+3bt6fu6dVhuQOrDpeCyyQdzlEBFq\/1CArtBPc0myUyA1a3Zk1\/gNgQPgEWDxHH9WuSD2U7yYAtvexH0KVyfJJNWanLhryUZjCuWefOnQsjR460wgi\/Rp1mTU4k3BxOev2KvUTAYrYTxmL5rp0o+2KTozqZmjIDVrdmTUagYWqxJXxT4RyivXHn27C\/+tsAf9pclDzhl08chm+2ytqSlzJ0s2LFCli2bJlWNlOaDHOaNXlum1y\/0qDutHw8NL\/zyhkD2EyAlUI3u3btUo4KF2dNHixYoy3hY9smwzmcrg\/uGyXu3uFOp7xqWFvychlMZvAXqlZbwqdOmgrnyHRheiIClnbsYOIEZjrl7bElLwfWDIwUW8LnpJtYv6roQsA2H9hUtIk9b4C1JS+368aBtcCBpMM5qkGt2sQe5lynDIjL2rLF7WfNgPRtzdQy6Xz9mkR2kxddqkynPAHWlrzcSREOrEUc4Kf8x0329xvUBFi+W8f0FZRJiTozYE2KIBP1uNCNCa62rfO8770s\/hh372vQoKbdOnwDex6SJoLoMiUl52AyxdkQ9doSvlcX8YY61LD4xNn7qkOXagN71kM6OnSFEL92UQdWbVaZK2hL+H4UkbMpTiqiLl15A6wuXUmPGAfWpDkaoT5bwvfrKo+9Rj3dX5cu9BDX\/\/B6ONW0OxdJE7p0RRgKvp84sCbN0Qj12RJ+UFfjhnLC0MV36vCkie7X1Ma+JzaIzrDvw9AVtm6\/8tqbz\/Go0jlz5sDy5cvdsS5JSsByuqEfKXFDOWEHtSqkk8Usp7B0JTVctMGK+1znz5\/vzmBKivOsHlvC1yElTignCl3kIeanTWQtBhuFLh1eB5URYPXbcM4rGD16tNUTJFzoJkicZt5HDeVEHdTkcMoqYKPSFVc62po1bkP4vepqDj4B0K0AjY2NoLqqw4E1CSmEryNqKCfOoCbA8qSJc\/5yGpxz8czwBCT8RRy64nQlVQcTbsHz2txOQL7uuutg3Lhx4poO\/JmfsujAGkfU8b6NEsqJO6gJsPykiSzEYOPSFVUSbcDKtZtcadz9rH5OKnw3depUqK6uFicromkub4J3YI0q5vjf8VCObqJE3EFNHmLsPc9ysg3YuHRFlUYRWEm79e\/f38ja1M9JJb9TlSWw\/upXv4IBA07fYhaV+Kx8Z0v4Yen\/5uNvwsZ3G8Th4K\/9y7DAz5OgCz3E++8bBXj4GoV00OFUetXTge2bKuBHV0lJialm0z03GI85xQuw6OGaWtakKi1MYB0\/fjxMmDDBGFPSrvjEiROwf\/9+6Nu3b6YPw67\/sBm+9uRuwZ47vlgOk68q92VVUnS1rFsCsG5J0QHix3tPgBN97IwBP7rKy8uhR48eRoaQUrNOnDjRyOmGs2bNgvr6+sL5Tvx3BCI3e\/3A+tRTT8GVV15phCE2Km1qaoJ9+\/YJa8HkzJwEbdPr\/gh1rx8QVW2aNdT3Co4k6Tr00xvFSYnc4XTWXz3heZFzErR61eFHF8rPlAytHpjG16k4WHkc188M3rBhgzODTY7GgLp1QzlJmMHUFa\/1K54ygYkTaT5J0hWm34WTIvC8YK+D0lRma5hGvMpy7Ylg5RlSzsGUBIfN1KEbykl6UNN9OvL6tfuXas0Q6lFr0nTpdj610A0PzWA4RnZmudDN3lzdtoYXXAVd0GxiUKsSJlS3rusCIEo5E3Tp9CM1sGJn5KQIOfHBJUUMNLbe0RkMYcrohHJMDeo\/jekgumornGOKriD+K698DDKHbaUdujhrkDjTfU\/a1etECVODmhL+OWDTTPg3RVeQ9NpoVgyv1NbWFiXsy\/e2ohcXnwceeCCo\/kTfO7Amys7YlfG1q+q8JpODmhL+baxfTdLlJxSlZlVdTMW9s+QMWrNmTWyBh6mAgzXMd1kv29zcDHv37oV+\/frlxgwmnt702Oufrl17QHXVxUWsjkJXmGQXW+tXB1YNRBFYV61aBUuWLIHNmzdrfOWK5IkDV111FSxatEgrNGcrHTETYEWh6pjBWAZjnQiaNB8CKwpz9uzZQqgVFRVpdsG1ZZADOPniJBwmjk7hnDTXr5kBKxKt2t+Kt8rhna34bubMmVBTUyMS7tN8ZLCGEWqa\/XRtReNAVJ9E2vtfMwXWaKw2\/5UDq3ke22whKlixzxTO4emIps4gdmDVGCXtAaxe125iTPrhhx+G7373uxCUu43e+kGDBom9wPLmCWRzt27dhGVUWVkp9g2HqU9DTJGLxAGrKpwjaL2mNvH8YatgpQGCgkNzN+v3s9Ka9Uw0g0kWKo88JZWEARf5F\/jl2JTKGQX8kZGo8WEcsGL1FM7Bn3E7HYZ1TMRfrYJVg4+ZKNKeNKsOWLnWJG25cePGwjbEu+++W8gNJzUOVsrJ\/vnPfy7ysQn8OvWhtkbNvXr1alE3tsFP84gzUOKCFdtOY\/3qwKohZT+wYp4q\/svbgxu58R89upoVy\/NdSjxRRTaDZbCStuWaNUp9O3bsEEfwPPjgg4lsqUwCrDycw9evSZrDmQIrn2FxRsZHPmLFBij8wMpzVW30LWqbCFTM\/pHBypcitEm\/tLTUc42JMtu+fbvIKgtas0atT3U2Fm8rKg\/ouyTAinWZXr9mBqy0IXzx4sUwY8aMgolkK8WQD4D2BNYgMxgdTuhjQJ7QQznbQZqVyvM1sE59BFbeJtaVVK54UmDl5rCJdMRMgJWfPih7CrN0Ir9zME0UWpSbt36aVTaDVWDVqY\/AGuTgiqphkwRrkXb9mxugpOJt0a0kttNlHqxZOpHfgbUYrMeOHQM8PGDYsGFKMzgsWHXro3JVVVWJOJmSBCsCs8g7\/PUroWP31uNo4q5fMwFWJIScD9wMHjJkiBgMSQkl7szrwDoRSCa4tsU1KJrEL7zwgvD6YgwVD6bz8garNKtufbI3OCkTGPuUNFixzp1TKgGdTl0Gj4DSa5vh1LHdEPc6jsyAFQlUpRsm6aJ3YI3KgTP7OxNg5eZwj1vuhJaz1womxsluyhRYszok2kOcNau8T6NfJsDKnU34c\/k\/DAfo+GdBTtTD1hxYNUaDA6sGk3JcxBRYeey19Isj4eyLtgsuRTWHrYLVKx9Vlnvc6zPijiMH1rgczPb3psCKVPOtdOdPmgafnHw+srPJKlhlEfpl0dgUtwOrTe6bb9skWLk5XNLrs8Icbj64KVLusAOrxlhoD2D1snIo9zfKHmLVGcwa7PbMAeanVPJ+8Zxhqj+MNWYarNzZdO5X\/xE69HhJdLPzwG8Jh5Pu48Cqwan2BFZVBpMGi5RFooAVv1m7dq2I28qWFs9m84q\/y+dE6\/TdNFixDzz2Wl71DYCSN0Kbww6sGtJ0YG0Nq2GeNmq0devWiRhrXV0dUN7wFVdcAXgXEF5ITeE2GayqXTNY5sc\/\/jEcOXIEBg8eXLRLB0VDKYxyfrB8VSeJMcoEkQZYsX889lo2qq8wh8M4mxxYY4IVPX6NL6\/UqCVbRUp6fxa6jfh2oVNB\/gKKgRMQScvNmzdPJEbs2bNHgBdPoKT7bvFn2oiBCROU0cR3zWAHvI7rkfvE98giwPjuH6xHd9+tLIm0wOoVe9VNRXRg1cCQn2blAtCoKjNF0NlxwWM72oBVPgCAbi+QwSHvTeW3xZM2xFMjEKy0Jc6rjAw67BTtwJJvT\/D6O34TRavid2mBlTub8Oc+c2bBiV2\/FDLQib1aBeuZELpBzfpB9e2ZAWGYjvSf93Iozaq6GlPeSM5NVxmsql0zN9xwg+82SHkPLAFeZQZH3TaXJlh57BUtm04D1wsZ6JjDVsEaZmDZLOvWrK1aS3U15pNPPilOfSAQcQePDFbVrpkgbUjt4roWnV\/V1dXidEvZkcR3boX1XKcJVhzHUWOvDqwas4AD6+m8bToaVl6zIhvxHa5Hg9asfNcMAZqOf+Fb7khL4\/+0NuaalZ8WEWcrZdpg5eYwj73i38+76fTSRB6aDqwOrIIDfksSOrUDD8LGZ9u2bUBrSfwdHUz9+\/cvnI\/Ez3rmpjP3BtOuGZVm5eX4mlXuI9\/kEaSh\/cRsA6xRYq8OrA6sGhzwdt5E9cBqNZpSIRtgRdL2V98uTGLhYNKIvTqwagyI9mAGB7HBS3M5sAZxzv+9KvbqdYypA6sGrx1YNZiU4yK2NCuyjJvDvf75fvi44XHBSVXs1YFVY5A5sGowKcdFbIJVdjb1mj4dmt5p9Q3Ix8A4sGoMsvYAVnLe0HlKxBbZO4t\/R8+r17m9shNITmrA78l0RqcUvxib6sWURXxUp4R4ZVqpQjfcUeWX2G8brEXadeoT8EnzcuUxMA6sDqyCAwSChoaGopv6VGClsA1+x8Em31SP7\/2u0cD3eOZW7969BYAx7XDatGkijuqV+0sAJI+zV9\/5hgAs43ekrW2wYv+4s2ng0leh8X+qhFz4MTAOrA6sBbBOmjRJhGBQs3nFPRGQeK4zJkLcc889sGDBgsIVnCpgqhxQCBzMXMLdNXSRlSwG1e4ZcnJhHjLtDiJtjFr4mWeeKeoPr9PvlMwsgBX7SjfSYWZT58uPi0R\/fCgV0YHVgbUIrPfeey888sgjRffQ0In7WJBrLAQnPnjyoO7WNAI7alRM9H\/00UfFNRhlZWVFkpDNXTJzp0+fLjKp5K18QRlMqomEGswKWPk2Oszb\/vDV60UXOw8cDV2HPggOrDHB+vGBTQWHgEZVmSnSqXRA0cZnPtgRRJRaiBdBcbCSVsQLrlGrEdiQMJ1rHDnYZdOXM0c2W2liwGQKPJ42DFi9TOqsgRX7Q6Ec1K7db\/p7OPr6bNFNdDZ9UnY57N27V2xPLCkpSW0sdWhpaWlJrbWYDfk5mBCstL6I2Uyqn8uxPFkzUVI8dorAqspyku9c5TtrVASpTnWQHUl0lQqZ4nxSoFRFXbD6OcOyCFaeN9xv3ssilEPHwHS5dp0DaxBK\/MCKhzfTNqegerL2\/pyLZhS6JIOVtNEll1wiNpyjI0llSnIHlJepSdq4T58+hbxhSraXc3pVjiDVxczYcdnJhGtuvoYO0qhZBCv2ia6PxAPCe027v6AMOnxmMjR0v8VpVj8gtZfQjTzYCSRoesqJ9MQv7rjBv8k3KGAdtbW1YmM6mtTylRp8bYoOJ3y4h1mWi27oRheoWH9W1qxEKw\/loHY9dfKFgkJovGgVVHxumDODvQDbXsHK46ETJkxooxWRX3K4RjaV5eNfVGYyatP33ntP1LV79+4iMcgmsi5YVdrYK9aaNbBy7UqHBBx6rlLw5WSPv4OeVy9xYG3PYM2aiZ5mf7IIVjlRosvnLy6Yw3EvuArL2zPGwRSWcFc+exzIIliRSzxR4sK6FvhwY1XoQ9aS4LYDaxJcdHUkwoGsghWJ44kSZTdPgqbf3CZojnPBVVimObCG5Zgrb4wDWQYrT5To9eBb0PTHe6DkaOuZw36nSiTJrFyDddWqVTBgwIAk+eHqssgBdGqNHTsW7n\/iObjly5+32BN105QoUTL8Fmj56nTo9s5YUVD3CNO4BOUSrAhSPNpk8+bNcel332eMA809L4IvTFgAa6YOzVjPig9Y6zB5FfTstwc+frda9FPnCNO4BOUSrBgjxEcOL8Rlhq3vMdcUd9n07Nkz1VCAaXrD0rXx3Qa4\/9eNcKq0J7zxg6vhgvO6mO5i6PopUQIuvAou+Mkr8NH668U2urD35YRuGAByC9Yzyfy1lRgeZcCE+SYKXZff9yrsPHQcqob1heqqS8I0l0pZOVGiY\/cDRXnDZ\/UcbqwfmQIr3\/Ss2iydZQdEHAlFGdRx2kvr2yh0Ta19C2q3tN5Mfuih1t0uWXtIu1KiRFqhnMyAlW\/tki8\/ImE5sGZt2Pr3JwpYX9neADc99rqo+LkpQ+HaQeWZI7rxdy\/B\/vtGiX7tkBPtAAAFs0lEQVT1mvoEpJUokRmwyjmkqlP8HFgzN259OxQFrFhh1k1hpKt+yTiA1\/5N0M8TJbxORExCcpkBq9e1EJh4jseN4ENgPdNCNs3NzbBy5UpxasOZtBaPStev95YAmsP4bP3eXyQxzhOtg+gav+1Hot4jg0ZCj1snQ\/f3W\/e8moq7ZgqsqguXli9fXgAren9nz57tQjaJDr1sVtbwjeXQ8dgB6P5f389mB\/8\/q2l8v8Mwvn8DvHiwDBa91wt+OLYFvj5igAjjmHhyBVZkAAL2TAnZmBCoq9MuB9AyMmUdZQqsqtvRuBlsVwyudccBuxzIDFjlkwriXHBkl6WudccBMxzIDFh1QjdmWOBqdRzIBwcyA1ZkV1BSRD5Y6nrpOGCGA5kCqx+J\/HgQ1XUOZtgTv1b5Kgq6D5Vq9qMrLzSrDu7OM12873ImnU26cgFWnjCBg5xu9KaT+eJDykwNqgOy+UFmfnTlhWaiETlIzsA804UTD14fUlNTI2444Kc82qYrF2CVj9aks3TxBPq8PVz4fnTlhWbs5\/PPPy+u+iCw5pkuv7Flm65cgFU+w9bvcqOsg5f33Y+uPNBMh37feOON8PDDDxfAmle6vE5spDFlm67cgJVfnKS6US3rIOUONLxTBq+9kGdxTpffu6zQSoeGY394jDyvdNEB67feeissXLhQWAt8zWqbLgfWlEa+6t5V28KPQzq\/K0d2MOWVLpJRRUWFuGUAH7w3iO6vtU1XbsCKjKMT4vNmBntdkGzbrIoKVvkiKxVYveSVZfNeZQZz2lDb2qQrF2CVzd48OZhUFxvzsI18MxyZ+1mmWQ5HET10OdbGjRvb3HiXB7pUd9jyTDr0ENuUVy7Ampcwhqyp+LUXqntjbIcCompW+TtZs+aZLu7xJTOYrhqxTVcuwIpMy0uCAB\/IXhqIOy1sBtlNgTVIXlmXJe9flpJYcgPWpAaWq8dxIK8ccGDNq+Rcv9sdBxxY253IHcF55YADa14l5\/rd7jjgwNruRO4IzisHHFjzKjnX73bHAQfWdidyR3BeOeDAmoLkMGkAc0zlR97YLB907te1rG9moKT4BQsWiH2hYR7azYMbHsrKysJ8ekaXdWBNQbxeh79h2mR9fb1IGg87KLMOVtqRg7uLojxIHz553LMchV6dbxxYdbgUs4wXWPkhcTgoZc3KM2ko7xa1lOrYEewiam+8tYAeOv6GtNyIESOgurr1PtGBAwcW9p\/i75TDvGvXLvHeL3OH90XFGlkz0smVuPVs7ty5hfonTJgAeK8RbkVT9WfSpEkQRTPHFFdmP3dgTUE0fseqcg3Jwbpv376iPaJy\/i3\/TpWDjO+XLl0qjifp06cP4HEyXlu\/5ElD\/l0+IUF15hJno7zRgtIuR40aJXZO0e\/l5eViwigtLRUTDeXgUl152rCRwjDK1\/2saTDERBt+YOXvduzYUThfSgar3K8gM5gDn8CKWo3MUg5A1MZ88zhvy2vt6QUk1c4V2WKQJwNsT0WPPEmYkE2e6nSaNQVpRQFrZWVlkVmL61q+\/vMCK4Jo9erVgioyV4PAitoXb5NXrZ29NiNg\/apTJlXg9gLrxIkTiyYPvv0M6w\/S4CmILlNNOLCmII4oZjD3oBIA5XWrvLcSQUoeZq6l44I1zGmSKo92VLDKtzSkIKpMN+HAmoJ4ojqYeNdU60gCa5A2CwJrkBmM611uQvuxLKgvOAmpTGWVpeA0azGnHVgtglUO3fg5mFSeYg5WDijy7DY0NBQ5mLzWrMgC2cEjH5laW1tb8B77nQIYZs0aZAa7NasDawrwLG7CKylCDo\/IgOTrT6yRr1tlj+rWrVsLiRdoLmNCATqNqqqqRBhG1o4yEOTQjZyw4dcXmaEqbzA3pXU1q\/MGO7CmDtb21mASa02cPGbMmAGLFy8uXKbd3vgo0+vM4PY+AgzR7zKYkmfs\/wEgwpcbWV4miAAAAABJRU5ErkJggg==","height":51,"width":68}}
%---
%[output:936cce49]
%   data: {"dataType":"text","outputData":{"text":"\n     Airfoil       CL      CD   Range (m)   Time (s)\n","truncated":false}}
%---
%[output:31d42e7b]
%   data: {"dataType":"text","outputData":{"text":"   FlatPlate     0.40   0.080       456.0       10.8\n   Eppler387     0.80   0.021       610.8       18.1\n    NACA2412     0.90   0.018       625.3       19.5\n","truncated":false}}
%---
%[output:0625a033]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAOsAAACwCAYAAADuQ5nhAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQvUVUUVxwclNYK0NEV8FGqmWVimS0MzK3y2sqLwQQiFRppaJolKWhrlA9+vUhJTA3WJC8VXpGVmamr2gLLUqMwHZmqZD0QzaP0m9mW+c8\/5zpxzZ+4599591nLJ931zZvbZs\/+znzMzYPny5cuNPsoB5UDtOTBAwVr7OVIClQOWAwpWFQTlQIdwQMHaIROlZCoHuhqs\/\/jHP8yYMWPMY4891pjpHXbYwcyYMcMMHjzY\/k7a8O85c+bY3\/GO\/Lzuuus2ScmFF15opk+fbvsZNWpUYSmS99NenDJlijn44IML99mNLwifhgwZYmbNmmXe\/e539\/nMr371q2bu3Lm58+DyOzn\/WXxL9u07Vsx56Fqw\/vjHPzaTJk1K5d1GG21kgQkQk2BNA2eyk5hgZayyi0BMQWl33y+++KKdv3vuuccOPXr0aHP66acXJkP6eeKJJxpzXrgTY4yCtQzXPN5xNaqrqX73u9+ZcePGmRdeeMHI7300q\/seK\/P73vc+c8EFF\/QBlbt6Z2kCIV3aJrWoLDCuYCa1sKsZhK6tttrKMOatt95qh0j26\/Zx5plnmquvvtokhVeEkffdxSzJbtoxjqvpkoKcXCh9tZk7lvTBu9CaZulkaT\/4h8YdNmyYfW\/x4sWNroU37pymLQaqWT2AFqJJmtBLv\/xt\/vz5jVU6D6y8lzSlpS\/RgGlmbX8C7wvWLOsgS+CELnexSPbB39Zcc00zYMCAhqZxgSp9ZAFM+sta7J566qnGgujOZVHN6IKF+Uozd7MAJePutddeBlC6bhB077jjjqk0ut+sYA2BRI8+ipipeWBFSPBPZSIZXswzwMrv+dnVVEmBTpLcn8\/an1ZOLkKiHegfTTd8+PBU2jAlBVzShywmshhtsMEGDV++P\/4Jv6Q9ffP9Akahaddddy1ltkJPck4WLlzYZwzhZ3+aVUzmNDdH3hOeuCa3LMAKVg+ghWiSprmyzMklS5b0CSiJ8IrZBVCTq7orzCNGjMjUvFnaJAusWdosy2RzzWAJmqXR5pqQSR8uSxOmmdMuSMQUvuyyy\/qYxUlfs79+suY6uShl+Z0+gMpbjCVGkVxgffoOIatF+ujKAFMR3y8mWLPAl1xMsqKeye\/I0qz4rDHAmrXYCB0HHHCAuf32242rlUX4kpH4In5rmlmedD342QdQSbAOGjSoyRKiLwVrkWUjYNusABNDJLVRHliTZrC0xw\/KMoPzPiVN84vg9ec3yXtJkzMLrGKiFzWD8+hPArG\/dJNoxQceeCA1\/ZIcK2lFJP9e1K9UMzhvNmvw9\/5SN65plmcmiVnsBimSq3yWWZuVgkkDa9oCk6VhfMFKDrhsgMk3op1sl8V3AVlycUymypILUlJTP\/fccw3Ql9GsjJe1IBRdCNot5l1pBmeZYvy+TFGEO7kEZnbZZRfzgx\/8IDN1k+en5UWDBQDrrbdewx\/mdxdddJE599xzG8Es8TezNKsUbLhmdrIPAUtyYcjL9ab5y8L3JGB989ppgR4XEMnAUFmwuhYWaTyepMnv07eCtd0c6PLxROhEGMsUgXQ5izrm87pas3bMLEQkNMtEL5r3jEiidu3JAQWrJ6M6uVnSxC0Sme3k7+422hWs3Taj+j1dy4GOA+vjjz\/etZOhH9b5HNhwww2jfUQUsPa3Jcn9WzI\/lyyYd7eywQGAetRRR5l77703GkO0Y+VAKxzYfvvtzWmnnWZigDY4WAnbT548uU8ujI+nVhMwHnrooXbHCo\/8m32KErbfeeedbZE19ab8293bSXJ\/7NixlhlUzXTKw+Jyzjnn1I7umHRJ30u2mWiWDVqn0FQNfOYhs8aD80yZdxlI3m+3nMg333HHHZ0BVoIZm222WeoGarQqHyIa023rAhnwAvpLLrmkz0ZxAWssZhSSqAKNH3nkEXPppZea4447zgwcOLDAm35Ny7oGvEdt79FHH12Krv60h8zV87udWhisqyx5xqz26N1m6RZ7+zEg0QqwDr5zupW1GBoui6jY8hlUs0oO7\/jjj089QQFwipbl\/+7PgHPatGmNbVvJn2kfmxmlJMPjpaVLl5onn3zS7hMNDVYAt99Bh5vFDy\/woCRsk2Gbb22uuvi8VEC0AtZWqVSwenAQsB544IFmv\/32s9vKqA5JlnC5WhdNu2jRImsiJzUpmnbq1Klm5syZ9kQHF6y33XZbW1dMj0\/vt0lMsN5\/\/\/3WNShrMpb9NjE1s+ZC6CqjWcvSJO8JWNstJ\/LNsTR6FM0quzBgHr4nO\/YBZNJELgvW8ePHmwkTJrQ6p217\/5VXXjFPP\/20GTp0aHDNumDBAnPkkUeadoNCADF79mz7XcmnKrrEZ8UMzqIt1sTjUlx++eXRzO8oYHXNYNecRduGMINhyLbbbhuL58H7ffnllw11vPhPoc1gzM2JEydWBtZbbrkl0wyugi4XrFm0BZ\/gFR3OmzfPHHvssZ0BVonoMklSRO6at5xmIGav+KxiFifN3m4KMMU0g6vyDfP8wqrocsEayxzNAnvsmEpQzcpHuBFfMYMlBRMqddPuSWh1JVawFkvdtMrvvIWk1f67BqwCWDF5kwXjrRRFxF65Yk2iglXBGkK2gmvWEERVtXLFol3BqmANIVsK1hBczOlDwapgDSFmCtYQXFSwNnFAA0zhC\/oVrArWUhzIC+IoWBWstlpHo8Er8VUVKBSszWtc7ACoatZSeqXYS+qzqs9aTGLSWytYQ3BRfVb1Wduw0UTBqmAtxQE1g9UM7ldwYvsEpaTW4yU1g9UM9hCT3CaqWXNZ1HoDBauCtXUpMkbBGoKL6rOqz1qFz5p2ZZ\/MRNXnzaoZnO0nVbWfNSuNVlVKCQ7l+dOx1ufY8tnQrMmLeTkHKfnIHSZ5lxZ1KjNi0a1msJrBIWTLgpXjWNglc8IJJ5jBgwfn9ov2pa3cLp37QqAGsVeuQGQ2daNgVbCGkC31WUNwUX1W9Vmr8FnbILulh1DNqj6rj\/B0vc\/qw4Sq2yhYFaw+MtgzYO3vxnD3UlwfpoVuo2BVsPrIVE+AVQ7p3n\/\/\/VNP1PdhVMw2VYK17Kn38OO1116zh3yvv\/76pU43jHXyfStzlQcITd1E3iInh3SfdNJJJi1108rkhni3KrBWfSFWf5cdVQUKBWu2lRNrC2dTNNg9eDsEwEL2URVYZdx2n3oP7+Tk+7oVHyhYawBW9Vnr4xcKWPu7ZEk1a\/N85S0kIRWI21dsZdJHs+ZdLBXrI337jc2MLDqqAoSC1Vcy+rbrGbBysZT6rH0nX8FaXHvVmWflloD8t2IrkyafNe3ainwy01ukXdvYiYd811nwqqItT3tVRZePNVJWnvPeaytYxQx+7LHHUukqkmeVvuhozpw59trGTr0+o86CVxVtCtYaBJjyVg\/fv6NBb7zxRntHq4C1U28+rwoQPlqiKtoUrF0CVjTo+eefb\/bcc09z1llnNcDaqTefVwUIBauvauihAFPoLXKAco899rAcnDZtWh+wduLN53W+xbsq2vJuF6+KLneB69qbz0NtPieoNH\/+fLvXNRlg6tSbz+t8i3dVtOnN581av+03n7dyrAvvTp482Rx22GG2XDENrHyibFp3zeJk27RIspij7b75nHHreot3VbQJWPXm85Wg7aibz0U7E1RyHzkG5q677urIm8\/VZ9U8q4\/33NbUjQ9BRdoktaOmbopw7\/9t6xp1rStdPjwrPgt+b3QVWPlkLYrwm3hpVVdQ1JUuBWsx+YrWOvbKlUW4msFqBvsIdWz51APTPGZBwapg9RATo2B1uBSbGapZfUSy3r50z5nBrl85Y8YMOzuXXHKJ4d8+5wr7T3mxlgrW5qNCqtL66rM2y25s+Wwyg8l9Ll682Jx99tnmiCOOsPnFUaNGmWSpYDGYhWkdmxmqWf3nScFaMVjdM5iGDx9uJk2a1AAraZepU6eamTNn2h00VTwKVtWsPnKXt5D49FGmTWz5bDopQjafJ8GaVlFU5oNaeSc2M1Sz+s9OHiCqMs97ymeVbWyuGTxixAgzZswYU\/URpQpW1aw+y0neQuLTR5k2seUzNXWTdmjalClTKj9LODYzVLP6i2geIFSzRj432H+qqmmpYFXN6iN5eQuJTx9l2sSWTy2K8JiVOmuJqmjLA0RVdPWMz5p3BpPI9ejRo9t+Nytjx1651Az2WLlWNFGwVpy6YXgCTFdeeWXjdAd+l7wDp6qcq4JVzWCf5SRvIfHpo0yb2PLpfci3m7p56qmnbM71hhtuKPNNpd+JzQzVrP5TkwcINYMjB5j6O5FfwTrWPL\/bqWbZoHX8JTpAy7qCoq509YzP6msGSy72iiuuCCCO\/l2oZlUz2Eda8hYSnz7KtIktn955Vor4qRFGw3LO0qxZs9p+LWRsZqgZ7C+ieYBQMziyGew\/VdW0VLCqZvWRvLyFxKePMm1iy6fmWT1mpc5aoira8gBRFV095bNmnVAIE4rcdeOBgcJNYq9cagb7T4mCtZlXseWzj2aVM4OHDRtWSdFDnqjEZoaCNW8GVv5dwVoxWN39rBzSXbdHwao+q49M5i0kPn2UaRNbPlM1q5wOUYbgmO\/EZoZqVv\/ZywOE+qxtiAYnr2X0n774LRWsqll9pCxvIfHpo0yb2PKZWm7YymXKyQBVsuhfD\/kuJgZ5gleVBqsrXT0VDS4mSn1bJ8sVkxsA9PqM4tytKyjqSpeCtbiMNd5wd+jozefFGVlXUNSVrp4Ca9qRLiJiZfKsLlj15nMFa3EOFH8jbyEp3qPfG231WSXPuvPOO5tx48Y1jiIte2Ca+K9nnHFG4+xhvfncb+KllQhe1i3eVd0wXle6XM3atTef85HJPKt7Uzkat8ip\/OKvbrfddn0uTw4B1vHjx5sJEyYUk\/oWWld1u7greLNnzzZDhw5t+oqqaBOw1o0uH561IAr9vtrWm8+TYMXHXLRokQVbkUO+04DKV4Yyg\/Xm85Uyg+lVxa3sAla9+XzlXLT95nMXUK42ZUvcHXfckXvfTTIC7C5FLvgFvKJpk4tBmiaP7RNkLZtVpUd8giVV0ZbnF1ZFlw\/PYmnW2PLZtOsmmX4BvHPnzjVDhgzJ3cOaV1usqZviYlJXUNSVrp4Ca3FxWvlG1o6dHXbYoaGRtSiiGIfrCoq60qVgXSFf1113nRk5cqReTFUMby21riso6kpXT4BVTFjsbh45xkWixNx1wzNnzhwFa0vwK\/ZyXUFRV7p6AqxuYMkN9ixcuNDmW8sURBQTy\/zWsR14DTDlz4G0ULA28yq2fNoAUzKo5AaKCC5VdQJ\/kh2xmaFgVbD6c6BCsMq9rLLpXKLArjncyoeEeFfBqlvkfOQoT+v79FGmTWz5bGjWNLBCMAURdXliM0M1q\/9M5wFC86yRNp+nHefilhr6T2HclgpW1aw+Epa3kPj0UaZNbPnsV7O6dbxliA\/9TmxmqGb1n7E8QKhmjahZSc1knRAhU1h1RFjBqprVZznJW0h8+ijTJrZ86iHfHrNSZy1RFW15gKiKLqYzjzaPKS\/VRMHqsC02M9QM9pfRPEAoWCOZwf5TVG1LBauawT4SmLeQ+PRRpk1s+VQz2GNW6qwlqqItDxBV0aVmsIdAt6NJ7JVLzWD\/WVSwNvMqtnyqZvWQzzpriapoU7AqWPuFTuyVSzWrx8q1oomCVcGqYE1woK6gqCtd6rP6L7hRW6pm1Wiwj4DlLSQ+fZRpE1s+1Wf1mJWq\/EIfLVEVbXmAqIouH555THmpJgpWh22xmaE+q7+MKljVZ1WfVX1W\/xUjo2XeQtLyABkdxFYmagZ7zFydTbqqaMsDRFV0qRnsIdDtaBJ75VIz2H8WFaxqBqsZrGaw\/4qhZnDLvCrdgXtIuHswuHSomlVTNz7Claf1ffoo0ya2fNbGZ027bpKrJw8++OAG32IzQ81gfxHNA4T6rF28Rc69B4cTFvViqv8Dp66gqCtdPjzzX5KKtYytTGqjWQHntGnTGif+J3+GbbGZEUuzrrLkGbPao3ebVzceaZYNWqeQBMQERTfSpWAtJF7lGic1adp9sALWK664wmy4YXgzI4vyxx9\/3IwdO9a8uNMUs2zQ2oU\/cJUlz5rBd04v9b68m\/XNrdDWjXQxOXk8KzyBni\/IXHA1agz5rJVmdW9WTwMrzDjqqKPMvffe68k+baYcaC8Htt9+e3PllVdGGbRWYM0zg+EAgOU\/fZQDdeQAGjWGVuVbawNWn5vP6zg5SpNyoF0cqA1YfVI37WKKjqMcqCMHagNWmJNXFFFHBipNyoF2caBWYG3XR+s4yoFO5ICCtRNnLTDNr776qhkwYIB53eteF7hn7S4kBxSsIbnZYX3997\/\/NTfddJMtRjnxxBPNXnvtVYsvIH7xm9\/8xqy++urmve99ry4iK2ZFwVoL8WwvEcuXLze\/\/e1vzTe\/+U3z\/PPPm2XLllnA7rTTTu0lJGW0n\/70p+ZrX\/uaefvb324eeughs\/nmm5sLLrjADBkypHLaqiZAwdqmGQAUr732mnnTm95kTc6qnmeeecaccMIJtrDk+OOPN7vssouZPHmy+dKXvmTk1vuqaOMWw8MOO8xq+fe85z3mX\/\/6lznooIPMZz\/7WfOxj32sKrJqM66CNfJUPP300xYMDz74oB1p+PDh5qyzzjLDhg2LPHJ69y+\/\/LL50Y9+ZD784Q+bN77xjSbtIu1KCDMmdfNGHS\/1roo\/CtaAnCf19Itf\/MJ8\/vOft9oTTUp55JZbbml\/x89HH320efbZZ61pN3jw4ICjp3eFX\/rLX\/7SpsVGjRpl3va2t\/XR7CwmkyZNsiZxuzUrJi\/VPpi8PA8\/\/LD5\/ve\/b4499li7kLz00kvmi1\/8otWsH\/rQh6Lzqu4DKFgDzpBs88OkBIjbbLONNS\/PPPNM+\/MZZ5xhwYwZ+v73vz+6OfzCCy+YI444wgr9OuusY3ct4Zvuueeeja+WYpSJEydaMLfrwW\/G4vjPf\/5j+ZMWiX700UfNl7\/8ZXPOOeeYjTfeuF2k1XYcBWvAqcEvPfTQQ81f\/\/pXqzkxddEKaFZ8xAMOOMD+RwT217\/+tTnppJMCjt7c1ezZs83Pf\/5zc95551kwXH755eauu+6yP6+22mr2BQHNVlttZQ488MCo9Lido9E\/97nPmb\/\/\/e\/WLfjABz7QNDb0\/upXv7KL3MCBAy2wV1llFbPqqqu2jc46DaRgDTgbBER+8pOfGHYPHXnkkeYjH\/mI1WSkIWbOnGne\/OY329FOP\/10q+2+8Y1vBBy9uSvGxAR2wfrPf\/7Talv3oR00nn322RYUoR\/4gpZ3tSMLyf33328DSddff725+OKLbfBNHnxrgk2f+MQnbEqJBe6UU04x3\/rWt6y\/3YuPgrWFWccf\/NnPfmZmzJhh1ltvPXPccceZt7zlLQZBnDt3rhXAJUuWWI2FX4ZviCmK4KF5N9hggxZGX\/kqGufSSy+1oETIP\/OZz9j0xyOPPGK1F2O\/4Q1vMPfdd5\/VqFgAmOKf\/OQnrSmO+T5lyhRz0UUXRTE3zz\/\/\/MYiJoC86qqrzLve9S7LAyK+o0ePtnTLIyYwlgq8gmYWvna4D0EmJUInClYPplLh8+1vf9uwV1EKBwAImpF0w8c\/\/nFzww03mHXXXdeatq+88oo9O+qDH\/ygDSw98cQTFkiYvviLgJcASqiHxeH222+35iLBq0MOOcSa46Q70Ghz5syx5i8aHbDcdtttlk4WmU022cQCfOrUqbZ9DK1FwOjmm2+2mhJ+JB9M9ZNPPtnSI9vLpE4c8\/3www83++yzj3n9618fimUd2Y+C1XPa0JSYt0Qr0Z6Ac\/78+Q3T8dprr7XaCi2A\/4UAot04YQIQixbzHM67GeY04Bw\/fnwjQES6g8ofNBURaMzej370o43AUjuDSoyPxYEGve6666y1QfrKfVj4cBvgK7lftD0+LTz+9Kc\/HXRh82ZsDRsqWFMmBU1z9dVXm+9+97v2r4COSCmRSSK8aE2Ai6nG3wAw2opqG9IP3\/ve96wWIIqJxkMQKT6IUQxBgIho6rbbbms1OZVJpDsQ\/JEjR1oNS5HB0KFDLR3QgKYHwPh\/73jHO4KIJYDD78WqYKGQtBQLAzwiKn7hhRfaSiQBpDvw73\/\/e8tf\/OZ2p5CCMKANnShYHSa7tbIIHOYXpis+F\/\/xYC4CYoAppvC\/\/\/1vK\/icYEH09wtf+EIj1xpzDqGXsQEA5iLABRDvfOc7LVgBKWYu2gkAk6vku2bNmmX23ntv6yuGWEAAGgsBATQ0IhFbfGi0KQEt\/GHM7ueee85aAZQ1UvfL7yWgBe3wde211zb77rtvTLZ1bN8K1hVTh7Zh9Ueg3ECGmJEIPxoCUxeQ0AYtSpEDgRnSNJht\/EdbTFE3uhlaQqDh3HPPtaDAtNxuu+2ahnADR3wH1gIPwRyCTiGAik\/M4kRKCn9cTNpNN93Uam8qpCi4wOcnTcMCSNCNRYUAkz7+HFCwruAVpi9mGP4lgJQVH+FDwDbbbDMLQHKoCCdgRZMQbUUTrLHGGuaHP\/yhDSSFivJmTSPaCk2JNiKdkfQB5b0\/\/elP5phjjrFWwfrrr+8vFQVaph1sBxAXLVpkA1rQgM+KJv3jH\/9o6cZ3RatrvW8BRtfpDKZiZIdpjbATGCIghDakcEEqZoj8ki5A4Eh5kIsUECKMHDdJ9JJAEv\/nug9MvJBR3mQQBq1P\/wg+fjHmLCfpSZ0xCwlansVlzTXXtKb5oEGDUn3EMhzEH+VhfHkwe0899VSrRSWSS\/SXxY2oN9Fy\/k+Ai4UN+gnWcfQs0etej\/AWmYee1qwIH+BEsEi\/oH0I1vztb38zu+66qxUmzEt8P8Asj+wGIdqKSRnywVQEaCLEsqBgRvI7sQDQSsn8JMEc2l1zzTW2LSWEWAEhAAEoiWxTPUTFEVVZaQ8+NAE4fFgxzTHBYxRbhOR7J\/TV02BlgihqQLjQrpi\/ABj\/Cw0AWLfYYotU3w6TjjYhTV78PwBI8EcKBCi3Q0NiyjIWP6PBCdhgomN2o2VD0pEFQhY2fNF58+bZQFtaOgp+EiiCJgJfAJVHwdr6ctDTYEVrUVBARREmHAAgqoqgIZCSU22dzf49YB6i3QWAmNwIPCkYdxMARfBoTDQYO2kkLeM\/UrGWbr4Wzc94u+++u91VJGka+IkVstZaa1mLBXpJ05ADnjBhQrEBtXUTB3oOrGgjVn\/80zFjxtgUDL4eGg3NiobFzMWE3G233azWDRE19ZU9iaYKAMnn3njjjVaLY+ZS6SPRZqnxBRChyheJMhPA4gF07m4Y\/HcCaVQikabCN+ZBu7PYiQlMwO2WW26xdcnQhr\/aq8X3vvPu065rwbpgwQIb+CFaKkeCkJ4BfPxMrSzHhlx22WU2mioalgANfyeyi\/8HWAB0zAeAEjwicAMo3Rv1+B0LB8D81Kc+ZRcOijHQtGhg\/G2OZQkBBuqWSU9hjkOT+KeyIwYeoS1xD9jAPn36dEsPfjR8HjFihF0AAfv+++9vQR0r4BZzPurad9eBFUCyou+4444WmFIgjkCResF0IzgjZXpoBNIw4i9S+8t2MfKQRIHpJ6bAUVAAEJcuXWrrevGXMS3JoVKIDyAxx1k0SMPgm1IZxd21ssk9hHAR1aZ\/XAH6ZgG48847LR8k2szPWB9sBmARIQrMQscCh3bFl2ah22OPPaL70CG+udP66BqwkmZBqFn90UAUOLAFC7MR\/w9zjhQCwvjWt77VCh3AfvLJJ81pp51mi\/QRRgSOrVwEdLLylyEmmcWD1AvajN04jA890Aj9LBqAEd+QyiPqkPFfqfAhV0mkNZR5jnktC1t\/0W1ypqRfGB9TV7S5HMCGL0vFlD5xONDxYHVLBBFgzDiJjCYLxKmcwdxEMNGwgIGc4OLFixvRyzhsbu5VaFu4cGHjTlqJTrN4sPDg98lWu5jVUJjVpHgwa\/ury8W8ZTEhEpxWMdUu3vXqOB0NViKUgA0\/Cp8JkzWpbZIF4kQrCZJQlypnIqGBEVZ3P2VogUD7YNb+4Q9\/sJqb+t0\/\/\/nP1gQmbykF9QRpCNxgCqPZ0WLjxo2zwa5WH2jA\/Mdfd\/nEYkVNMz56GgjZHE6Qi78ThWahi8mrVr+zW9\/vaLAyKfha+KLUx4pWwJxk9wYCtfXWW1u\/T\/y\/r3\/964b9qZiXbIDGfCZiGSoPKHtXSVcASHnwOUkH4QPefffddt8o+2HxTSmEkLQRxQcEZvgb74csKCCYxjgsAJjeUiyBSY75D\/\/SUkDuRde4FQTcNBXT\/iWh48GKOYlZRoCG4gFuCCcpT8IeLUBwCABh8qKxNtpoIwtutAk\/o7VCVPiIz8y5QRQ1fOUrX7Fjs0jgHwM6NChpF\/FNCSbhj5I2otiAwAxRaFImbn1yKLGgJhfTGnOWIBFRXanMYlwCbd\/5znfsUSvyyBY8TGXa84Ra2EJ9V6\/00\/FgZaLY\/YIZRzSVPZ1ENZOBDnw\/gk\/Js35anej+fGb6lvJAgjikkqSAgN05BJig59Zbb7VAwbzE3OUbWk3FADIWMjkYDVqIkhM4I4oL8CQAR12zbPcj\/0wbygn5NkoX+ZlorwviVvmm7xfnQFeAVVZ\/TGLMTTmYzGUHYKFEkM3jrQJB+sWcxrekBDDpM0MTxRXQQt6UrXQULkiEGc2OyYkZTLE9eVNAE6oIg4WAzQakecRyQDui8QEeZ0ZhBeAu4DdjtvM9aH\/MZQJalF7SDi0PbfpUy4GuACsslKMtSca3M\/iRVjyBmUswBhMY0CD4aE5MdQJcpJHQYPjSpGP4Oz+7G9tbFQsx\/fFFZSuaG80ld8oCw0JD0IncKvtOoYWFjYIRouqANVSKqNVv6vX3uwasTCSmLpoBjREzR+oKjaRgiDATmUaTse0OkOCTitlLeR5BGTQt\/ikmKD61HLiNTwvdaDA3MOUjoJirFDEkD8pOnhuFpkSDs2jgOuArQ2Of+CsvAAABzklEQVSydFBOyPcZW9u0jwNdBVY0AsJIGgQt1a4HM5diATQUASM31+vSgM+MmU5gC780RO4UnxjTlggzUWS2sbnnHxH1pVxQzGs0O7ldzG93pw4mO2kuznIKEXBrF+97aZyuAisTxzEiaLmYJYJJARGfmQAOASP3DhsWECLUXF\/BHlnyuQA67UjOsoJHsIrFCZChNQEoPig8SJrXUt8b60Dvst+g7+VzoOvAmv\/JcVrgIxKRxvylZC8ZJcZEJm1EEIyATdqRnGUpk4gzRf2caEgF1F\/+8hdbTA89VCbx4DczPn\/Hl04LxJWlQd+LzwEFa0Aeo7U4wAwth5mZdoo8Pi4amB0qIW+RQ4PiA5Nj5iAyuSyZs5A40ZBdSKRgqC2msAHAcjypPp3DAQVrwLmSw7Px\/UIWXPiQiCmO5iQqLreySYE9VUscqcpmAXK9mMuh0lc+tGmbMBxQsIbhY6OXKnxmGVw20WNyu9c3AlrOlSJaHPv4l8Ds1O4cDihYu0wc3EuxQkSbu4w9Hf05CtaOnr5m4uXkxeStbF32mT35OQrWLpx2ygrZDNDO9FUXsrF2n6Rgrd2UKEHKgXQOKFhVMpQDHcIBBWuHTJSSqRz4H9sTzljT0gDzAAAAAElFTkSuQmCC","height":51,"width":68}}
%---
%[output:2f69d925]
%   data: {"dataType":"text","outputData":{"text":"\nBest: NACA2412  ,  Range = 625.3 m  ,  Time = 19.5 s\n","truncated":false}}
%---
