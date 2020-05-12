function display_hist_angles(thetas)
% display_hist_angles Display the histogram of a group of orientations
%
% INPUT
%   thetas: Vector of angles in radians

%--------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%--------------------------------------------------------------------%
%  1.1. CREATE A FIGURE
figure('Name', 'Histograms');
%  1.2. CONVERT THE ANGLES TO SEXAGESIMALS
thetas = thetas*180/pi;

%--------------------------------------------------------------------%
% 2. GET HISTOGRAM OF THE POSITIVE ANGLES
%--------------------------------------------------------------------%
%  2.1. GET POSITIVE ANGLES
thetas_pos = thetas>=0;
%  2.2. COMPUTE HISTOGRAM
[n_POS,xout_POS] = hist(thetas(thetas_pos),20);

%--------------------------------------------------------------------%
% 3. GET HISTOGRAM OF THE NEGATIVE ANGLES
%--------------------------------------------------------------------%
%  3.1. GET NEGATIVE ANGLES
thetas_neg = thetas<0;
%  3.2. COMPUTE HISTOGRAM
[n_NEG,xout_NEG] = hist(thetas(thetas_neg),20);

%--------------------------------------------------------------------%
% 4. PLOT RESULTS
%--------------------------------------------------------------------%
%  4.1. COMPUTE THE MAXIMUM
n = max(n_POS, n_NEG);
%  4.2. PLOT RESULT
subplot(121)
bar(xout_POS,n_POS)
axis([min(xout_POS)-1   max(xout_POS)+1  0  max(n)+5])
xlabel('Positive angles')
%  4.3. PLOT RESULT
subplot(122)
bar(xout_NEG,n_NEG)
axis([min(xout_NEG)-1   max(xout_NEG)+1  0  max(n)+5])
xlabel('Negative angles')


end