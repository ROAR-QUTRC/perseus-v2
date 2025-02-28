import type { RequestHandler } from "@sveltejs/kit";
import { exec } from "child_process";

export const POST: RequestHandler = async ({ request }) => {
  const { domainId } = await request.json();

  exec(`export ROS_DOMAIN_ID=${domainId}`, (error, stdout, stderr) => {
    if (error) {
      console.error(`exec error: ${error}`);
      return;
    }
    if (stderr) {
      console.error(`stderr: ${stderr}`);
      return;
    }
  });

  return new Response(JSON.stringify(domainId));
};

export const GET: RequestHandler = async (req) => {
  return new Promise((resolve, reject) => {
    exec("hostname -I", (error, stdout, stderr) => {
      resolve(
        new Response(
          JSON.stringify(
            stdout.length === 1
              ? "localhost"
              : stdout.substring(0, stdout.length - 2),
          ),
        ),
      );
    });
  });
};
