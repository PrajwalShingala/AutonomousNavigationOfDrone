function [a] = capVelocity(a)
	a
	a(1) = min(a(1), 2);
	a(1) = max(-2, a(1));
	a(2) = min(a(2), 2);
	a(2) = max(a(2), -2);
end
