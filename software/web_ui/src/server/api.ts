import { remultSveltekit } from "remult/remult-sveltekit";
import { Layout } from "../shared/Layout";

export const api = remultSveltekit({
  entities: [Layout],
  admin: true, // enable admin UI
});
