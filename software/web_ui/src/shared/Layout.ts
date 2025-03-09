import type { WidgetType } from "$lib/scripts/state.svelte";
import { Entity, Fields } from "remult";

@Entity("layout", {
  allowApiCrud: true,
})
export class Layout {
  @Fields.uuid()
  id!: string;

  @Fields.string()
  title?: string;

  @Fields.string()
  description?: string;

  @Fields.boolean({ defaultValue: () => false })
  isLocked?: boolean;

  @Fields.json()
  widgets: Array<{
    name: string;
    x: number;
    y: number;
    w: number;
    h: number;
    state?: string;
  }> = [];
}
