import { readdir } from "fs/promises";
import { join } from "path";
import { json } from "@sveltejs/kit";

export async function GET() {
  const dir = join(process.cwd(), "static/maps");
  const files = await readdir(dir);
  const glbs = files.filter((f) => f.endsWith(".glb")).sort();
  return json(glbs);
}
