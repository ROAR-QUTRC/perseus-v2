export type ImuDataType = {
  angular_velocity: { x: number; y: number; z: number };
  linear_acceleration: { x: number; y: number; z: number };
};

export const updateHUD = (
  ctx: CanvasRenderingContext2D,
  canvas: HTMLCanvasElement,
  imuData: ImuDataType,
) => {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#0a0a0a";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  const pi = Math.PI;
  const pi2 = pi * 2;
  const piHalf = pi / 2;
  const w = canvas.width;
  const h = canvas.height;
  const c: { x: number; y: number } = {
    x: w / 2,
    y: h / 2,
  };
  const mainRadius = w / 4 > h / 2 - 100 ? h / 2 - 100 : w / 4;

  ctx.fillStyle = "";
  ctx.strokeStyle = "#62d252";
  ctx.lineWidth = 2;
  ctx.beginPath();
  // draw a circle at the center of the canvas
  ctx.arc(c.x, c.y, mainRadius, 0, Math.PI * 2);
  ctx.stroke();

  // draw arcs on the left and right sides of the circle
  ctx.beginPath();
  ctx.arc(
    c.x,
    c.y,
    mainRadius * 1.05,
    pi * -0.75 - piHalf,
    pi * -0.25 - piHalf,
  );
  ctx.arc(
    c.x,
    c.y,
    mainRadius * 1.3,
    pi * -0.25 - piHalf,
    pi * -0.75 - piHalf,
    true,
  );
  ctx.arc(
    c.x,
    c.y,
    mainRadius * 1.05,
    pi * -0.75 - piHalf,
    pi * -0.75 - piHalf,
  );
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(c.x, c.y, mainRadius * 1.05, pi * 0.25 - piHalf, pi * 0.75 - piHalf);
  ctx.arc(
    c.x,
    c.y,
    mainRadius * 1.3,
    pi * 0.75 - piHalf,
    pi * 0.25 - piHalf,
    true,
  );
  ctx.arc(c.x, c.y, mainRadius * 1.05, pi * 0.25 - piHalf, pi * 0.75 - piHalf);
  ctx.stroke();

  const horizonTilt = imuData.angular_velocity.x;
  const horizonHeight = imuData.angular_velocity.z * 10;
  // Horizon line
  ctx.beginPath();
  ctx.moveTo(
    c.x + mainRadius * Math.cos(horizonTilt),
    c.y + mainRadius * Math.sin(horizonTilt),
  );
  ctx.lineTo(
    c.x + mainRadius * Math.cos(horizonTilt + pi),
    c.y + mainRadius * Math.sin(horizonTilt + pi),
  );
  ctx.stroke();

  for (let i = 0; i < 50; i++) {
    //calculate the end points of the lines based on the horizon height and tilt
    const line = {
      x1:
        c.x +
        (mainRadius / ((i % 2) + 1) / 2) * Math.cos(horizonTilt) -
        40 * Math.sin(-horizonTilt) * (i - 10) +
        40 * Math.sin(-horizonTilt) * horizonHeight,
      y1:
        c.y +
        (mainRadius / ((i % 2) + 1) / 2) * Math.sin(horizonTilt) -
        40 * Math.cos(-horizonTilt) * (i - 10) +
        40 * Math.cos(-horizonTilt) * horizonHeight,
      x2:
        c.x +
        (mainRadius / ((i % 2) + 1) / 2) * Math.cos(horizonTilt + pi) -
        40 * Math.sin(-horizonTilt) * (i - 10) +
        40 * Math.sin(-horizonTilt) * horizonHeight,
      y2:
        c.y +
        (mainRadius / ((i % 2) + 1) / 2) * Math.sin(horizonTilt + pi) -
        40 * Math.cos(-horizonTilt) * (i - 10) +
        40 * Math.cos(-horizonTilt) * horizonHeight,
    };
    // check if the line is inside the main circle
    if (
      Math.abs(line.x1 - c.x) > mainRadius - 10 ||
      Math.abs(line.y1 - c.y) > mainRadius - 10
    ) {
      continue;
    }
    if (
      Math.abs(line.x2 - c.x) > mainRadius - 10 ||
      Math.abs(line.y2 - c.y) > mainRadius - 10
    ) {
      continue;
    }
    ctx.beginPath();
    ctx.moveTo(line.x1, line.y1);
    ctx.lineTo(line.x2, line.y2);
    ctx.stroke();
  }

  // Draw heading direction
  const headingDirection = imuData.angular_velocity.y;
  ctx.beginPath();
  ctx.moveTo(c.x, c.y + mainRadius * 1.3);
  ctx.stroke();
};
